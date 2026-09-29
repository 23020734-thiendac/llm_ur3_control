# UR3 LLM Control — Bài thực hành 02

Điều khiển robot UR3 trong Gazebo bằng câu lệnh tiếng Việt hoặc tiếng Anh. LLM chỉ hiểu yêu cầu và sinh JSON plan gồm các robot skill; LLM không sinh joint trajectory. MoveIt 2 lập kế hoạch, kiểm tra collision và gửi quỹ đạo đến controller của UR3.

Sinh viên: **Ngo Thien Dac**  
MSSV: **23020734**

## 1. Chức năng

Package hỗ trợ ba chế độ:

1. **Cơ bản:** mỗi câu lệnh xử lý một khối theo chuỗi pick → place → home.
2. **Nâng cao:** một câu lệnh xử lý cả ba khối theo mã sinh viên:
   - Zone A → blue_cube
   - Zone B → red_cube
   - Zone C → yellow_cube
3. **PLUS:** thêm khối cam chiếm Zone A, B hoặc C. Robot chuyển cam sang temporary_zone trước khi đặt khối đúng màu.

Các skill chính:

~~~text
pick(object)
place(object, zone)
home()
~~~

Gripper mô phỏng là gripper song song hai ngón. Hai ngón mở và đóng bằng cùng một trajectory để chuyển động đồng thời.

## 2. Kiến trúc

~~~text
Câu lệnh người dùng
        ↓
command.py
        ↓ /llm/command
llm_planner_node
        ↓
LLMPlanner → 9Router → LLM
        ↓ JSON plan
task_validator.py
        ↓
skill_executor.py
        ↓
robot_skills.py
        ↓
MoveIt 2
        ↓
controller → UR3/Gazebo
        ↓
/llm/task_result → terminal
~~~

Các file quan trọng:

~~~text
launch/llm_robot.launch.py          Khởi động Gazebo, UR3, controller, MoveIt và planner
ur3_llm_control/llm_planner.py     Gọi 9Router qua /v1/chat/completions
ur3_llm_control/planner_node.py    ROS node nhận lệnh và điều phối task
ur3_llm_control/task_validator.py  Kiểm tra JSON plan và mapping MSSV
ur3_llm_control/skill_executor.py  Chạy skill theo thứ tự, fail-stop
ur3_llm_control/robot_skills.py    MoveIt, gripper, attachment và scene
ur3_llm_control/simulation.py      Sinh world SDF và gripper URDF
config/scene.yaml                  Bàn, cube, zone, cam và vùng tạm
config/student_config.yaml         Tên, MSSV và nhiệm vụ cá nhân
prompts/planner.txt                Prompt và schema JSON cho LLM
test/test_core.py                  Unit test validator, executor và HTTP client
~~~

## 3. Yêu cầu môi trường

- Ubuntu 22.04.
- ROS 2 Humble.
- Gazebo Fortress / Ignition Gazebo.
- MoveIt 2.
- Các package Universal Robots: ur_description và ur_moveit_config.
- 9Router đang chạy local.
- Model 9Router hỗ trợ Chat Completions và trả JSON text.

## 4. Clone repository và build

~~~bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/23020734-thiendac/llm_ur3_control.git
~~~

Build:

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src/llm_ur3_control \
  --ignore-src -r -y

colcon build \
  --packages-select ur3_llm_control \
  --symlink-install

source install/setup.bash
~~~

Nếu clone vào đường dẫn khác trong src, thay src/llm_ur3_control bằng đường dẫn thực tế.

## 5. Cấu hình 9Router

Mở giao diện:

~~~text
http://localhost:20128
~~~

Trong 9Router:

1. Vào Providers.
2. Kết nối một provider.
3. Chọn model.
4. Ghi lại Model ID chính xác.
5. Vào Endpoint & Key.
6. Lấy endpoint và tạo API key local nếu cần.

Endpoint mặc định:

~~~text
http://localhost:20128/v1
~~~

Không ghi API key vào source code, YAML hoặc GitHub.

Trong terminal chạy launch:

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export NINE_ROUTER_BASE_URL='http://localhost:20128/v1'
export NINE_ROUTER_MODEL='MODEL_ID_TRONG_9ROUTER'

read -rs -p '9Router API key: ' NINE_ROUTER_API_KEY
export NINE_ROUTER_API_KEY
echo

export ROS_DOMAIN_ID=68
~~~

Nếu 9Router không yêu cầu key, bỏ hai dòng read và export API key.

## 6. Khởi động mô phỏng

Terminal 1:

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=68

ros2 launch ur3_llm_control llm_robot.launch.py
~~~

Chờ dòng:

~~~text
[llm_planner]: READY: nhập lệnh bằng ros2 run ur3_llm_control command
~~~

Không chạy hai mô phỏng trong cùng một ROS_DOMAIN_ID.

## 7. Chạy mức cơ bản

Terminal 2:

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=68
~~~

Chạy từng câu và chờ kết quả:

~~~bash
ros2 run ur3_llm_control command "Đưa khối màu đỏ vào vùng B."
ros2 run ur3_llm_control command "Move the blue cube to zone A."
ros2 run ur3_llm_control command "Hãy lấy khối màu vàng và đặt nó vào ô C."
~~~

Mapping MSSV 23020734:

~~~text
red_cube    → zone_b
blue_cube   → zone_a
yellow_cube → zone_c
~~~

Log đúng:

~~~text
USER COMMAND:
Đưa khối màu đỏ vào vùng B.

LLM PLAN:
pick(red_cube)
place(red_cube, zone_b)
home()

EXECUTION:
pick(red_cube)                         SUCCESS
place(red_cube, zone_b)                SUCCESS
home()                                 SUCCESS
TASK SUCCESS
~~~

## 8. Chạy mức nâng cao

~~~bash
ros2 run ur3_llm_control command --timeout 1800 \
  "Arrange all objects according to my student ID."
~~~

Plan phải gồm:

~~~text
pick(blue_cube)
place(blue_cube, zone_a)
home()

pick(red_cube)
place(red_cube, zone_b)
home()

pick(yellow_cube)
place(yellow_cube, zone_c)
home()
~~~

Executor chạy theo thứ tự Zone A → Zone B → Zone C và về home sau mỗi vật.

## 9. Chế độ PLUS và khối cam

Khối cam ban đầu nằm trên mặt bàn ở phía trên bên phải. Vùng temporary_zone nằm phía dưới cam.

Mở teleop:

~~~bash
ros2 run ur3_llm_control command --teleop
~~~

Phím điều khiển:

~~~text
a  Đưa cam vào Zone A
b  Đưa cam vào Zone B
c  Đưa cam vào Zone C
h  Reset robot và cảnh
q  Thoát teleop
~~~

Ví dụ tạo tình huống cam chiếm Zone B:

~~~text
Nhấn b trong cửa sổ teleop.
~~~

Sau đó chạy plan nâng cao:

~~~bash
ros2 run ur3_llm_control command --timeout 1800 \
  "Arrange all objects according to my student ID."
~~~

Executor tự chuyển cam:

~~~text
pick(orange_cube)
place(orange_cube, temporary_zone)
home()
~~~

Sau đó tiếp tục đặt red_cube vào zone_b.

Đưa cam vào vùng tạm riêng:

~~~bash
ros2 run ur3_llm_control command --orange-temp
~~~

Hoặc:

~~~bash
ros2 run ur3_llm_control command "Đưa khối cam vào vùng tạm thời."
~~~

## 10. Lệnh sai và kiểm tra an toàn

Mapping bắt buộc:

~~~text
Zone A → blue_cube
Zone B → red_cube
Zone C → yellow_cube
~~~

Ví dụ:

~~~bash
ros2 run ur3_llm_control command "Đưa khối màu đỏ vào vùng A."
~~~

Kết quả:

~~~text
INVALID_OBJECT
~~~

Các lệnh nhiễu cũng không làm robot chuyển động:

~~~bash
ros2 run ur3_llm_control command "Đưa khối màu tím vào vùng A."
ros2 run ur3_llm_control command "Xoay robot 90 độ."
~~~

Validator kiểm tra trước khi gọi MoveIt. Nếu một skill lỗi, executor dừng task và yêu cầu reset bằng phím h.

## 11. Kiểm tra plan không chạy robot

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export NINE_ROUTER_BASE_URL='http://localhost:20128/v1'
export NINE_ROUTER_MODEL='MODEL_ID_TRONG_9ROUTER'
export NINE_ROUTER_API_KEY='API_KEY_LOCAL'

ros2 run ur3_llm_control command --plan-only \
  "Đưa khối màu đỏ sang vùng B."
~~~

Kết quả hợp lệ:

~~~text
PLAN VALID — chưa thực thi robot
~~~

## 12. Chạy test

~~~bash
cd ~/ros2_ws/src/llm_ur3_control
source ~/ros2_ws/install/setup.bash
python3 -m pytest -q test/test_core.py
~~~

Hoặc:

~~~bash
python3 -m unittest discover -s test -v
~~~

Test kiểm tra schema JSON, mapping MSSV, plan một vật và nhiều vật, trùng object/zone, INVALID_OBJECT, executor fail-stop, PLUS, HTTP contract và scene Gazebo.

## 13. Bảo mật

Không commit:

~~~text
API key
.env
log/
build/
install/
__pycache__/
.pytest_cache/
~~~

API key nhập bằng biến môi trường:

~~~bash
read -rs -p '9Router API key: ' NINE_ROUTER_API_KEY
export NINE_ROUTER_API_KEY
~~~

Trước khi commit:

~~~bash
git status
git diff --cached
~~~

## 14. Chẩn đoán lỗi

| Lỗi | Cách kiểm tra |
|---|---|
| Chưa đặt NINE_ROUTER_MODEL | Export model trước khi launch |
| HTTP 401/403 | Kiểm tra API key local |
| HTTP 404 | Kiểm tra URL có /v1 |
| Không kết nối được 9Router | Kiểm tra 9Router chạy tại port 20128 |
| NOT_READY | Chờ launch báo READY |
| PLANNING_FAILED | Kiểm tra collision, TF, controller và scene |
| ZONE_OCCUPIED | Bật PLUS để chuyển cam sang temporary_zone |
| INVALID_OBJECT | Kiểm tra mapping MSSV |
| LLM trả markdown | Chọn model tuân thủ JSON tốt hơn |

## 15. Tài liệu đi kèm

~~~text
docs/bao_cao_thuchanh2_23020734.docx
docs/so_do_luong_xu_ly_ur3.png
docs/so_do_luong_xu_ly_ur3_don_gian.png
~~~

## License

Apache-2.0.
