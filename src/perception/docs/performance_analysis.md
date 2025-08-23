# Perception System Performance Analysis

## API Performance Test Results (640x480 Resolution)

### ReferingAPI (Chassis Camera - Object Detection)
- **Input Size**: 640x480 pixels
- **Average Latency**: 3.25 seconds
- **Theoretical Max FPS**: 0.31
- **Recommended Rate**: 0.15 Hz
- **Interval Between Calls**: ~6.7 seconds

### GraspAnythingAPI (Hand Camera - Grasp Detection)  
- **Input Size**: 640x480 pixels
- **Average Latency**: 1.55 seconds
- **Theoretical Max FPS**: 0.64
- **Recommended Rate**: 0.3 Hz
- **Interval Between Calls**: ~3.3 seconds

## Frame Rate Design Rationale

### Conservative Rate Setting (50% Utilization)
We set the processing rate at 50% of theoretical maximum to ensure:
1. **Network Stability**: Handle API response time variations
2. **System Load**: Leave CPU/memory headroom for other processes
3. **Queue Management**: Prevent image queue overflow
4. **Error Recovery**: Time for retry on API failures

### Final Configuration
```xml
<!-- Chassis Detector: 0.15 Hz -->
<param name="rate" value="0.15"/>

<!-- Hand Grasp Detector: 0.3 Hz -->
<param name="rate" value="0.3"/>
```

## Performance Optimization Tips

1. **Image Preprocessing**
   - Images are already 640x480, no resizing needed
   - Consider JPEG compression for network transfer

2. **Queue Management**
   - Queue size = 2 (drop old frames, keep latest)
   - Async processing prevents blocking

3. **API Call Optimization**
   - Batch processing not available
   - Consider caching for repeated queries

4. **Monitoring**
   - Watch for API timeout errors
   - Monitor queue overflow warnings
   - Track actual processing times

## System Requirements

- **Network**: Stable internet connection required
- **CPU**: < 30% usage per node
- **Memory**: < 500MB per node
- **Bandwidth**: ~200KB per API call (JPEG compressed)

## Future Improvements

1. **Local Model Deployment**: Eliminate network latency
2. **GPU Acceleration**: Use local GPU for inference
3. **Dynamic Rate Adjustment**: Adapt rate based on load
4. **Result Caching**: Cache detection results for static scenes