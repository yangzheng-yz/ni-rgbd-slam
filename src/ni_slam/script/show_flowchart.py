from graphviz import Digraph
from IPython.display import display

option = "depth2normal"

if 0:
    g = Digraph('G', format='png')

    g.attr(rankdir='TB', size='16,16')

    g.attr('node', shape='box')
    g.node('A', '1. Initialize variables\n\n'
                '- vertexMapz, vertexMapx_, vertexMapy_\n'
                '- utanMap_x, utanMap_y, utanMap_z\n'
                '- vtanMap_x, vtanMap_y, vtanMap_z')

    g.node('B', '2. Compute vertex maps\n\n'
                '- vertexMapx_ = vertexMapxx .* vertexMapz\n'
                '- vertexMapy_ = vertexMapyy .* vertexMapz')

    g.node('C', '3. Compute U-tan and V-tan\n\n'
                '- Calculate differences for utanMap_*\n'
                '- Calculate differences for vtanMap_*')

    g.node('D', '4. Compute and normalize normal vectors\n\n'
                '- Calculate normalsEig_* using cross product\n'
                '- Normalize normalsEig_* vectors')

    g.node('E', '5. Convert normal vectors to OpenCV format\n\n'
                '- Convert Eigen matrices to cv::Mat\n'
                '- Merge channels into _normalsCV')

    g.node('F', '6. Blur normal map\n\n'
                '- cv::blur(_normalsCV, _normalsCV, cv::Size(_cellSize, _cellSize))\n'
                '- Split channels back into separate cv::Mats')

    g.node('G', '7. Normalize normal vectors again\n\n'
                '- Convert cv::Mat back to Eigen matrices\n'
                '- Normalize normalsEig_* vectors')

    g.node('H', '8. Store final normal map\n\n'
                '- Convert Eigen matrices to cv::Mat\n'
                '- Merge channels into _normalsCV')

    g.edges(['AB', 'BC', 'CD', 'DE', 'EF', 'FG', 'GH'])

    g.view()
elif 1:
    dot = Digraph("Callback Function Flowchart", format="png")

    dot.node("A", "1. 开始")
    dot.node("B", "2. 计时并输出处理第n帧信息\nnth_frame")
    dot.node("C", "3. 更新ros_header并计算时间差\ntime_diff")
    dot.node("D", "4. 尝试将深度图像从ROS消息转换为OpenCV矩阵\ncv_bridge::toCvShare")
    dot.node("E", "5. 进行类型转换（如果需要）\nimD.convertTo")
    dot.node("F", "6. 调整图像尺寸\ncv::resize")
    dot.node("G", "7. 检查是否初始化\ninitialized")
    dot.node("H1", "8.1 计算法线图\nEfficientDepth2NormalMap")
    dot.node("H2", "8.2 初始化rotation_rel和rotation_key")
    dot.node("I1", "9.1 计算法线图\nEfficientDepth2NormalMap")
    dot.node("I2", "9.2 初始化AnglesMap")
    dot.node("I3", "9.3 计算旋转矩阵\nEfficientNormal2RotationMat")
    dot.node("I4", "9.4 更新rotation_rel")
    dot.node("J", "10. 检查是否初始化\ninitialized")
    dot.node("K", "11. 重新投影点云\nreproject")
    dot.node("L", "12. 获取响应\nget_response")
    dot.node("M", "13. 计算PSR\nget_psr")
    dot.node("N", "14. 优化关键点云位姿\nrefine_keyclouds_poses")
    dot.node("O", "15. 发布位姿\npublish_poses")
    dot.node("P", "16. 发布twist（如果需要）\npublish_twists")
    dot.node("Q", "17. 发布增量关键帧协方差（如果需要）\npublish_incremental_keypose_cov")
    dot.node("R", "18. 检查PSR、有效点数和旋转角度\npsr, valid_points")
    dot.node("S", "19. 输出分辨率、计时信息和PSR\nresolution, time_use, psr")
    dot.node("T", "20. 显示可视化结果（如果需要）\nshow")
    dot.node("U", "21. 增加帧数\nnth_frame++")
    dot.node("V", "22. 结束")

    dot.edges([("A", "B"), ("B", "C"), ("C", "D"), ("D", "E"), ("E", "F"), ("F", "G"), ("G", "H1"), ("H1", "H2"), ("G", "I1"), ("I1", "I2"), ("I2", "I3"), ("I3", "I4"), ("I4", "J"), ("J", "K"), ("K", "L"), ("L", "M"), ("M", "N"), ("N", "O"), ("O", "P"), ("P", "Q"), ("Q", "R"), ("R", "S"), ("S", "T"), ("T", "U"), ("U", "V")])
    dot.view()

