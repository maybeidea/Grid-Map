> 历史设计记录：不作为当前代码、参数或运行方式的依据。当前入口见 [README](../../README.md)。

GridMap 只有在当前位置连续融合满 stationary_frames 后才发布 HeightMap。这样会浪费掉前面的积累，stationary_frames作为上限，在选择比如5帧作为下限，位置移动且满足下限即融合进heightmap和esdf并发布；

将“HeightMap 尚未到达”视为等待状态，不要立即进入观测失败流程，提高鲁棒性；

