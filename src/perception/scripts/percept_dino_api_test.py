import base64
import copy
import json
import math
import os
import sys
import time
from io import BytesIO

import cv2
import numpy as np
import requests
from dds_cloudapi_sdk import Client
from dds_cloudapi_sdk import Config as DDSConfig
from dds_cloudapi_sdk.tasks.v2_task import V2Task, create_task_with_local_image_auto_resize
from mmengine.config import Config
from PIL import Image
from pycocotools import mask as coco_mask


def flatten(lst):
    result = []
    for item in lst:
        if isinstance(item, list | tuple):
            result.extend(flatten(item))  # 递归展开
        else:
            result.append(item)
    return result


class ReferingAPI:
    def __init__(self, cfg):
        self.uri = cfg.uri
        self.status_uri = cfg.status_uri
        self.token = cfg.token
        config = DDSConfig(self.token)
        self.client = Client(config)

    def forward(self, text, rgb, **kwargs):
        body = {'model': 'DINO-XSeek-1.0', 'image': None, 'prompt': {'type': 'text', 'text': None}, 'targets': ['bbox']}
        # task = create_task_with_local_image_auto_resize(
        #     api_path=self.uri,
        #     api_body_without_image=api_body,
        #     image_path='path/to/infer/image.jpg',
        # )
        # breakpoint()
        pil_img = Image.fromarray(rgb)

        # 创建BytesIO对象并保存图像数据
        buffer = BytesIO()
        pil_img.save(buffer, format='PNG')
        buffer.seek(0)  # 将指针移回起始位置

        # Base64编码
        img_bytes = buffer.getvalue()

        b64 = base64.b64encode(img_bytes).decode('utf-8')
        b64 = f'data:image/jpg;base64,{b64}'
        body['image'] = b64
        body['prompt']['text'] = text
        task = V2Task(api_path=self.uri, api_body=body)
        self.client.run_task(task)
        # print(task.result)  # {'objects': [{'bbox': [507.8738098144531, 602.8898315429688, 634.7514038085938, 817.6101684570312]}]}
        return task


class GraspAnythingAPI:
    def __init__(self, cfg):
        self.server_list_fp = cfg.server_list
        if not os.path.isfile(self.server_list_fp):
            if self.server_list_fp.startswith('/'):
                tmp = './' + os.path.basename(self.server_list_fp)
                if not os.path.isfile(tmp):
                    raise Exception(f'file not found: {self.server_list_fp} or {tmp}. {cfg=}')
                else:
                    print(f'load cfg from  {tmp}')
                    self.server_list_fp = tmp
        with open(self.server_list_fp) as f:
            api_dict = json.load(f).get('backends', {})
            self.server_list = api_dict
        self.model_name = cfg.model_name
        assert self.model_name in self.server_list, f'{self.model_name} not in {self.server_list}. {cfg=}'
        self.url = self.server_list[self.model_name] + '/generate'

    def forward(self, rgb, depth=None, **kwargs):
        # print('url:', URL)
        if isinstance(rgb, str):
            img = Image.open(rgb).convert('RGB')
        elif isinstance(rgb, np.ndarray):
            img = rgb
        elif isinstance(rgb, bytes):
            img = cv2.imdecode(
                np.frombuffer(img, dtype=np.uint8),  # 从 bytes 转成 uint8
                cv2.IMREAD_COLOR,  # 读取为 BGR 格式（OpenCV 默认）
            )
        else:
            raise NotImplementedError(f'{type(rgb)}')
        h, w = img.shape[:2]
        max_dim = max(h, w)
        # 计算上下和左右的padding量
        top = 0
        bottom = max_dim - h - top
        left = 0
        right = max_dim - w - left

        padded_img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_REPLICATE)

        _, img_encoded = cv2.imencode('.jpg', padded_img, [cv2.IMWRITE_JPEG_QUALITY, 30])
        img_bytes = img_encoded.tobytes()
        response = requests.post(self.url, files={'img_source': img_bytes})

        # breakpoint()
        objs = response.json()  # [bs, obj]
        bs = len(objs)
        assert bs == 1, f'{bs=}'
        obj_num = len(objs[0])
        if obj_num == 0:
            return objs, padded_img
        obj0 = objs[0][0]
        # breakpoint()
        if 'dt_mask' in obj0 and obj0['dt_mask'] is not None:
            kernel = np.ones((5, 5), np.uint8)
            size = (max_dim, max_dim)
            for obj in objs[0]:
                dt_mask = obj['dt_mask']
                obj['dt_rle'] = dt_mask
                m = coco_mask.decode(dt_mask)
                m = cv2.resize(m, dsize=size)  # , interpolation=cv2.INTERPOLATION_BILINEAR)
                m = cv2.dilate(m, kernel, iterations=1)
                if 0:
                    m = cv2.erode(m, kernel, iterations=1)
                m = m[:h, :w].astype(bool)
                obj['dt_mask'] = m

        ret = objs
        if kwargs.get('bag', False):
            ret = self.extract_bag(ret, rgb, depth, **kwargs)
        if kwargs.get('use_touching_points', True):
            ret = self.post_process(ret, rgb, depth, **kwargs)
        
        return ret, padded_img

    def post_process(self, ret, rgb, depth, **kwargs):
        for obj in ret[0]:
            if 'dt_mask' in obj:
                # breakpoint()
                m = obj['dt_mask']
                affs = obj['affs']
                touching_points = []
                for rb in affs:
                    # breakpoint()
                    xc, yc, w, h, angle2 = rb
                    # angle2 = angle2 % math.pi  # in [0, 180] and w >=h which is the same as affordance
                    # assert w >= h and angle2 >= 0 and angle2 <= math.pi, f'{w=} {h=} {angle2=}'

                    p1, p2, p3, p4 = cv2.boxPoints(((xc, yc), (w, h), angle2 * 180 / math.pi))
                    c1 = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]
                    c2 = [(p4[0] + p3[0]) / 2, (p4[1] + p3[1]) / 2]
                    c1 = [int(x) for x in c1]
                    c2 = [int(x) for x in c2]

                    line = np.zeros_like(m, dtype=np.uint8)
                    line = np.ascontiguousarray(line)  # must
                    # breakpoint()
                    cv2.line(line, c1, c2, color=255, thickness=1, lineType=cv2.LINE_8)
                    line = line.astype(bool)
                    # breakpoint()
                    union = line & m
                    points = np.argwhere(union > 0)

                    if len(points) < 2:
                        ps = [c1, c2]
                        msg = '.mask'
                    else:
                        pt1 = tuple(points[0][::-1])
                        pt2 = tuple(points[-1][::-1])
                        pt1 = [int(x) for x in pt1]
                        pt2 = [int(x) for x in pt2]
                        ps = [pt1, pt2]
                        # print(f'ps={ps}')
                        rb[0] = (pt1[0] + pt2[0]) / 2.0
                        rb[1] = (pt1[1] + pt2[1]) / 2.0
                        rb[2] = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
                        rb[3] = 30
                    touching_points.append(ps)
                obj['touching_points'] = touching_points
        # breakpoint()
        return ret

    def extract_bag(self, ret, rgb, depth, **kwargs):
        ret0 = ret[0]
        img_h, img_w = rgb.shape[:2]  # (720, 1280)
        for obj in ret0:
            mask = obj['dt_mask']
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                return
            largest_contour = max(contours, key=cv2.contourArea)
            guess_mask = np.zeros_like(mask)
            guess_mask = np.ascontiguousarray(guess_mask)  # this is must
            guess = cv2.minAreaRect(largest_contour)
            (cx, cy), (w, h), angle = guess
            if w < h:
                w, h = h, w  # 交换宽高
                angle += 90  # 角度调整
                angle = angle % 180  # in [0, 180] and w >=h which is the same as affordance

            guess = [(cx, cy), (w, h), angle]

            p1, p2, p3, p4 = cv2.boxPoints(guess).astype(int)  # 4 points in (x,y)
            # breakpoint()
            guess = flatten(guess)  # xc, yc, w, h, angle(degree)
            guess[4] = guess[4] / 180 * math.pi
            # breakpoint()
            # cv2.drawContours(guess_mask, [p4], -1, 255, -1)

            c1 = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]
            c2 = [(p4[0] + p3[0]) / 2, (p4[1] + p3[1]) / 2]
            c1 = [int(x) for x in c1]
            c2 = [int(x) for x in c2]
            c3 = [0.2 * c2[0] + 0.8 * c1[0], 0.2 * c2[1] + 0.8 * c1[1]]  # near c1
            c4 = [0.2 * c1[0] + 0.8 * c2[0], 0.2 * c1[1] + 0.8 * c2[1]]  # near c2
            c3 = [int(x) for x in c3]
            c4 = [int(x) for x in c4]
            if c3[0] >= img_w or c3[1] >= img_h or c4[0] >= img_w or c4[1] >= img_h:
                aff = None
            else:
                if mask[c3[1], c3[0]] and (not mask[c4[1], c4[0]]):  # near c2 part is background
                    aff = [*c2, max(abs(c4[0] - c2[0]), abs(c4[1] - c2[1])), 30, guess[4]]
                elif (not mask[c3[1], c3[0]]) and mask[c4[1], c4[0]]:  # near c1 part is background
                    aff = [*c1, max(abs(c3[0] - c1[0]), abs(c3[1] - c1[1])), 30, guess[4]]
                else:
                    aff = None
                    print('can determine')
            # breakpoint()
            # dic = copy.deepcopy(obj[0])
            # dic['affs'] = aff
            # dic['scores'] = 1.0
            # breakpoint()
            if aff is not None:
                obj['scores'][-1] = 1.0
                obj['affs'][-1] = aff

                # breakpoint()
        return ret

    def vis(self, objs, rgb, padded_img, img_fp, to_save=True):
        import supervision as sv

        objs = objs[0]
        # breakpoint()
        dt_mask = [obj['dt_mask'] for obj in objs]
        dt_bbox = [obj['dt_bbox'] for obj in objs]

        N = len(dt_mask)

        lines = np.array([obj['touching_points'] for obj in objs])  # (obj#, aff#, 2, 2)
        lines = lines.reshape(-1, *lines.shape[2:])
        # breakpoint()
        rgb = cv2.polylines(rgb, lines, isClosed=False, color=(0, 0, 255), thickness=2)
        vis_img = np.asfortranarray(rgb)
        masks = np.array(dt_mask).astype(bool)  # must be bool!!!!
        bboxes = np.array(dt_bbox)
        # breakpoint()
        detections = sv.Detections(
            xyxy=bboxes,
            mask=masks,
            class_id=np.array([1] * N),
        )
        labels = ['o'] * N

        pil = Image.fromarray(vis_img)
        annotated_frame = pil
        box_annotator = sv.BoxAnnotator()
        annotated_frame = box_annotator.annotate(annotated_frame, detections=detections)
        label_annotator = sv.LabelAnnotator()
        annotated_frame = label_annotator.annotate(annotated_frame, detections=detections, labels=labels)
        annotated_frame = label_annotator.annotate(annotated_frame, detections=detections)
        mask_annotator = sv.MaskAnnotator()
        annotated_frame = mask_annotator.annotate(annotated_frame, detections=detections)
        if to_save:
            fp_out = img_fp.replace('.png', '.jpg').replace('.jpg', '_vis.jpg')
            annotated_frame.save(fp_out)
            print(f'save vis to : {fp_out}')
        return np.array(annotated_frame)

    def preprocess(self, image):
        # Implement preprocessing steps like resizing, normalization, etc.
        pass  # Placeholder for actual preprocessing logic

    def to_coco(self, dts, height, width):
        imgs = []
        anns = []
        cats = [{'id': 0, 'name': 'object'}]
        for frame_id, dt in enumerate(dts):
            img = dict(id=frame_id, width=width, height=height, file_name=frame_id)
            imgs.append(img)
            objs = dt[0]
            for obj in objs:
                obj['image_id'] = frame_id
                obj['category_id'] = 0
                if 'dt_mask' in obj and obj['dt_mask'] is not None:
                    mask = obj['dt_mask']
                    mask = np.asfortranarray(mask)
                    rle = coco_mask.encode(np.asfortranarray(mask))
                    rle['counts'] = rle['counts'].decode('utf-8')
                    obj['dt_mask'] = rle
            anns.extend(objs)

        coco = dict(images=imgs, annotations=anns, categories=cats)
        return coco

    def demo_video(self, fp, fp_out, fps=None, func=None, **kwargs):
        if not os.path.isfile(fp):
            print(f'file not found: {fp}')
            return
        cap = cv2.VideoCapture(fp)
        if fps is None:
            fps = cap.get(cv2.CAP_PROP_FPS)
        if not cap.isOpened():
            print('fail to open: {fp}')
            sys.exit(1)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 编码器
        out = None
        frame_id = 0
        dts = []
        while cap.isOpened():
            if frame_id % kwargs.get('log_interval', 10) == 0:
                print(f'process on frame {frame_id}')
            if kwargs.get('debug', False) and frame_id > kwargs.get('max_frame', 50):
                break
            frame_id += 1

            ret, fra = cap.read()
            if not ret:
                break

            if func is not None:
                fra = func(fra)
            det, padded_img = self.forward(rgb=fra, depth=None)
            dts.append(det)
            # breakpoint()
            img_vis = self.vis(objs=det, rgb=fra, padded_img=padded_img, img_fp=None, to_save=False)
            if out is None:
                height, width, _ = img_vis.shape
                out = cv2.VideoWriter(fp_out, fourcc, fps, (width, height), True)
            out.write(img_vis)
        cap.release()
        out.release()
        print(f'read {fp} and write to {fp_out}: {frame_id} frames')

        coco = self.to_coco(dts=dts, width=width, height=height)
        fp_out2 = fp_out.replace('.mp4', '.json')
        fo = open(fp_out2, 'w')
        json.dump(coco, fo, indent=2)
        fo.close()
        print(f'write coco json to {fp_out2}')
        return coco


if __name__ == '__main__':
    cfg = Config()
    cfg.server_list = r'/comp_robot/common_config/server_grasp.json'
    cfg.model_name = 'full'
    service_graspany = GraspAnythingAPI(cfg)

    cfg = Config()
    cfg.uri = r'/v2/task/dino_xseek/detection'
    cfg.status_uri = r'/v2/task_status'
    cfg.token = 'c4cdacb48bc4d1a1a335c88598a18e8c'
    service_referany = ReferingAPI(cfg)
    img_fp = 'dino_test.jpg'
    rgb = cv2.imread(img_fp)

    if 1:
        objs, padded_img = service_graspany.forward(rgb=rgb)
        print(objs)
        service_graspany.vis(objs, rgb=rgb, padded_img=padded_img, img_fp=img_fp)

    if 1:
        task = service_referany.forward(rgb=rgb, text='水瓶')
        print(task.result)

        task = service_referany.forward(rgb=rgb, text='纸巾')
        print(task.result)

    if 0:
        fp = r'/comp_robot/dino-3D/grasp/Ours/xarm/0730/grasp4track_cam1.mp4'
        fp_out = fp.replace('.mp4', '_vis.mp4')
        func = None
        service_graspany.demo_video(fp=fp, fp_out=fp_out, func=func)
