# Kết quả kiểm thử — 26/09/2026

Sinh viên: **Ngo Thien Dac — 23020734**. Phạm vi: mức cơ bản.

## Đã kiểm tra thành công

- Build package bằng colcon vào `/tmp/ur3_llm_check/install` và workspace `~/ros2_ws/install`.
- `python3 -m unittest discover -s test -v`: **10/10 test đạt**.
- Python compileall cho module và launch: đạt.
- Executable `ros2 run ur3_llm_control command --help` trong workspace chính: đạt.
- Khởi chạy Gazebo Fortress không GUI, UR3, controller, TF, MoveIt, collision scene: READY.
- Ba nhiệm vụ skill thực sự qua MoveIt và controller trong Gazebo: tất cả SUCCESS.
- Đọc lại `/get_planning_scene`: không còn vật attached; robot ở home (sai lệch mỗi joint < 0.01 rad).
- Tọa độ cuối cube trong MoveIt, frame world:

| Cube | Vùng | Tọa độ tâm (m) |
|---|---|---|
| blue_cube | A | (0.29, 0.12, 0.1175) |
| red_cube | B | (0.29, 0.00, 0.1175) |
| yellow_cube | C | (0.29, -0.12, 0.1175) |

- Đã gửi câu tiếng Việt qua executable command vào ROS node khi chưa cấu hình model:
  nhận `TASK FAILED: Chưa đặt NINE_ROUTER_MODEL`, không thực thi robot.

## Log thật của bài kiểm thử robot

Lệnh kiểm thử: `python3 test/smoke_robot.py --execute`, trên ROS_DOMAIN_ID=174,
launch `gazebo_gui:=false launch_rviz:=false`. Kế hoạch cố định chỉ dùng trong test.
Đây **không phải** log LLM hiểu câu tự nhiên.

```text
SKILL INTEGRATION TEST (NO LLM): red_cube -> zone_b
EXECUTION:
pick(red_cube)                           SUCCESS
place(red_cube, zone_b)                  SUCCESS
home()                                   SUCCESS
TASK SUCCESS
SKILL INTEGRATION TEST (NO LLM): blue_cube -> zone_a
EXECUTION:
pick(blue_cube)                          SUCCESS
place(blue_cube, zone_a)                 SUCCESS
home()                                   SUCCESS
TASK SUCCESS
SKILL INTEGRATION TEST (NO LLM): yellow_cube -> zone_c
EXECUTION:
pick(yellow_cube)                        SUCCESS
place(yellow_cube, zone_c)               SUCCESS
home()                                   SUCCESS
TASK SUCCESS
ALL THREE SKILL INTEGRATION TASKS PASSED; LLM NOT TESTED
```

## Phần chưa được kiểm chứng / còn cần làm

- Chưa có model/provider 9Router của sinh viên: chưa gọi LLM thật, chưa kiểm thử toàn tuyến
  câu tự nhiên → LLM → robot. Test HTTP hiện dùng server mock riêng.
- Chưa kiểm tra cửa sổ Gazebo/RViz bằng mắt hoặc ghi hình demo; bài thử trên chạy headless.
- Gripper hai ngón mô phỏng động học, không mô phỏng lực kẹp và ma sát.
- Chưa quay video, xuất báo cáo PDF có ảnh thực tế, tạo GitHub Public hoặc upload Drive.
- Chưa kiểm thử UR3e; bài thử robot dùng UR3.

## Các sửa lỗi đã xác minh trong quá trình thử

Đường tự do hạ vật có thể sinh tư thế không phù hợp: chuyển hạ/nâng/mang vật sang Cartesian,
chỉ thực thi đường đủ 100% và kiểm tra trạng thái sau time parameterization.
Ràng buộc vai/khuỷu khi tiếp cận giúp tránh các cube khác.
Giảm chiều cao nâng và đưa vùng đặt vào trong tầm với UR3 có gripper hai ngón.
Đồng bộ attach/detach MoveIt với Gazebo; lỗi skill làm dừng chuỗi.

Mô phỏng thử được dừng sau khi kiểm tra; khởi chạy lại theo README để bắt đầu cảnh mới.
