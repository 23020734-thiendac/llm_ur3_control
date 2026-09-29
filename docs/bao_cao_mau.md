# Báo cáo thực hành 02 — bản mẫu cần bổ sung kết quả thật

**Sinh viên:** Ngo Thien Dac  
**MSSV:** 23020734  
**Phạm vi:** mức cơ bản, một vật mỗi câu lệnh.

## Cấu trúc và luồng xử lý

Package `ur3_llm_control`, ROS 2 Humble, Gazebo Fortress, MoveIt 2, 9Router.
Node `llm_planner` nhận `/llm/command`, gọi LLM, kiểm tra toàn bộ kế hoạch, gọi executor.
Kết quả được xuất ở terminal và `/llm/task_result`.
LLM không tạo joint trajectory. MoveIt lập kế hoạch và điều khiển qua trajectory controller.

## Robot skills

- `pick(object)`: tới trên vật → hạ gripper hai ngón → attach collision object → nâng vật.
- `place(object, zone)`: tới trên vùng → hạ vật → detach → cập nhật vật trong scene → nâng gripper hai ngón.
- `home()`: MoveIt đưa robot về cấu hình home đã khai báo.

Mọi bước chuyển động kiểm tra kết quả MoveGroup, lỗi làm dừng chuỗi.
Validator chỉ cho phép đúng ba bước, object/zone trong whitelist, không có tham số thừa.
Gripper hai ngón được mô phỏng động học bằng TF và Gazebo set_pose; chưa có mô phỏng lực kẹp/ma sát.

## Nhiệm vụ cá nhân

`XX = 34`, `P = 34 mod 6 = 4`.
Zone A nhận xanh dương; B nhận đỏ; C nhận vàng.
Demo cơ bản bằng ba câu riêng, một vật mỗi lần.

## Kiểm thử kỹ thuật đã thực hiện

10 kiểm thử logic đạt. Bài thử riêng các skill trong Gazebo/MoveIt (không qua LLM)
đã thành công cho đỏ→B, xanh dương→A, vàng→C, mỗi nhiệm vụ có home.
Xem `kiem_thu.md` để đối chiếu log và tọa độ cuối.
Điều này chưa chứng minh khả năng hiểu ngôn ngữ của LLM thật.

## Kết quả chạy thử LLM — cần điền sau demo

| Câu lệnh | Kế hoạch LLM thực tế | Kết quả robot thực tế |
|---|---|---|
| Đưa vật màu đỏ sang vùng B. | Chưa ghi nhận | Chưa ghi nhận |
| Move the blue cube to zone A. | Chưa ghi nhận | Chưa ghi nhận |
| Hãy lấy khối màu vàng và đặt nó vào ô C. | Chưa ghi nhận | Chưa ghi nhận |

Bổ sung: ảnh cảnh ban đầu, terminal có MSSV/LLM PLAN, robot đang mang vật,
cảnh sau place, log SUCCESS; một ví dụ đầu vào không hợp lệ bị từ chối.

## Liên kết nộp bài

- Video Drive Public: **CHƯA CÓ**
- GitHub Public (repo riêng hoặc nhánh assignments_2): **CHƯA CÓ**

Sau khi bổ sung minh chứng, mở Markdown bằng trình soạn thảo và xuất PDF.
Không dùng bảng mẫu này để khẳng định đã chạy thành công.
