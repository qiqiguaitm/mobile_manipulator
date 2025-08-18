#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
诊断joint_states发布问题
"""

import rospy
from sensor_msgs.msg import JointState
import collections
import time

class JointStateDiagnostics:
    def __init__(self):
        rospy.init_node('joint_state_diagnostics')
        
        # 记录各个发布者
        self.publishers = collections.defaultdict(lambda: {
            'count': 0,
            'last_time': None,
            'last_stamp': None,
            'timestamps': collections.deque(maxlen=10)
        })
        
        # 记录时间戳问题
        self.timestamp_issues = collections.defaultdict(int)
        self.last_timestamps = {}
        
        # 订阅joint_states
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, 
                                    queue_size=100, tcp_nodelay=True)
        
        rospy.loginfo("开始诊断joint_states...")
        
    def joint_state_callback(self, msg):
        """分析joint_states消息"""
        # 获取发布者信息
        publisher_id = msg._connection_header.get('callerid', 'unknown')
        current_time = time.time()
        msg_stamp = msg.header.stamp.to_sec()
        
        # 更新发布者统计
        pub_info = self.publishers[publisher_id]
        pub_info['count'] += 1
        
        # 计算发布频率
        if pub_info['last_time'] is not None:
            time_diff = current_time - pub_info['last_time']
            if time_diff > 0:
                freq = 1.0 / time_diff
                pub_info['freq'] = freq
        
        # 检查时间戳问题
        if len(msg.name) > 0:
            joint_name = msg.name[0]
            
            # 检查时间戳是否更新
            if pub_info['last_stamp'] is not None:
                stamp_diff = msg_stamp - pub_info['last_stamp']
                if stamp_diff <= 0:
                    self.timestamp_issues[publisher_id] += 1
                    rospy.logwarn(f"时间戳问题: {publisher_id} - 差值: {stamp_diff:.6f}s")
            
            # 检查与其他发布者的时间戳冲突
            if joint_name in self.last_timestamps:
                for other_pub, other_stamp in self.last_timestamps[joint_name].items():
                    if other_pub != publisher_id:
                        time_gap = abs(msg_stamp - other_stamp)
                        if time_gap < 0.001:  # 1ms内认为是冲突
                            rospy.logwarn(f"时间戳冲突: {publisher_id} vs {other_pub} - 间隔: {time_gap*1000:.3f}ms")
            
            # 更新记录
            if joint_name not in self.last_timestamps:
                self.last_timestamps[joint_name] = {}
            self.last_timestamps[joint_name][publisher_id] = msg_stamp
        
        # 更新发布者信息
        pub_info['last_time'] = current_time
        pub_info['last_stamp'] = msg_stamp
        pub_info['timestamps'].append(msg_stamp)
        pub_info['joints'] = msg.name
        
    def print_report(self):
        """打印诊断报告"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("Joint States 诊断报告")
        rospy.loginfo("="*60)
        
        # 发布者统计
        rospy.loginfo("\n发布者统计:")
        for pub_id, info in self.publishers.items():
            freq = info.get('freq', 0)
            issues = self.timestamp_issues.get(pub_id, 0)
            joints = info.get('joints', [])
            
            rospy.loginfo(f"\n发布者: {pub_id}")
            rospy.loginfo(f"  - 消息数: {info['count']}")
            rospy.loginfo(f"  - 频率: {freq:.1f} Hz")
            rospy.loginfo(f"  - 时间戳问题: {issues}")
            rospy.loginfo(f"  - 关节数: {len(joints)}")
            if joints:
                rospy.loginfo(f"  - 关节: {', '.join(joints[:6])}")
        
        # 问题总结
        total_issues = sum(self.timestamp_issues.values())
        if total_issues > 0:
            rospy.logwarn(f"\n⚠️  发现 {total_issues} 个时间戳问题!")
            rospy.loginfo("\n可能的解决方案:")
            rospy.loginfo("1. 检查是否有多个节点发布相同的joint_states")
            rospy.loginfo("2. 确保每个发布者使用rospy.Time.now()更新时间戳")
            rospy.loginfo("3. 考虑使用joint_state_publisher来合并多个源")
            rospy.loginfo("4. 降低发布频率或使用消息过滤")
        else:
            rospy.loginfo("\n✅ 未发现时间戳问题")
        
        # 检查多发布者情况
        if len(self.publishers) > 1:
            rospy.logwarn(f"\n⚠️  检测到 {len(self.publishers)} 个发布者!")
            rospy.loginfo("建议使用joint_state_publisher统一管理")
        
        rospy.loginfo("\n" + "="*60)

def main():
    diag = JointStateDiagnostics()
    
    # 运行10秒收集数据
    rospy.loginfo("收集数据10秒...")
    start_time = time.time()
    
    while not rospy.is_shutdown() and (time.time() - start_time) < 10:
        rospy.sleep(0.1)
    
    # 打印报告
    diag.print_report()
    
    # 查找相关节点
    rospy.loginfo("\n查找joint_states发布节点...")
    import subprocess
    try:
        result = subprocess.check_output(['rostopic', 'info', '/joint_states'], 
                                       stderr=subprocess.STDOUT)
        rospy.loginfo("\n" + result.decode())
    except:
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass