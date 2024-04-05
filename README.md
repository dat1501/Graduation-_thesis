"# -n-t-t-nghi-p-" 
"# Graduation-_thesis" 
" Đồ án tốt nghiệp về Ứng dụng sóng LoRa vào khu vườn Thông Minh 
- Các thiết bị Cần thiết Trong Đồ án Stm32F103c8t6(2), ESP32(1), Module LoRa Sx1278(2)
- Mục tiêu: kiểm soát khu vườn thông minh thông qua nhiệt độ, độ ẩm của không khí, độ ẩm đất,ánh sáng qua đó có thể điều khiển các thiết bị chấp hành để giúp khu vườn có đầy đủ môi trường thuận lợi để thực vật phát triển
1 nguyên lý cơ bản:
  - Stm32 thứ nhất sẽ xử lý việc đọc dữ liệu từ các cảm biến và truyền dữ liệu của cảm biến cho con Stm32 thứ 2 đồng thời cũng nhận dữ liệu từ con stm32 thứ 2 để xử lí yêu câu bật tắt các thiết bị chấp hành nếu người dùng yêu cầu"
  - Stm32 thứ 2 sẽ xử lý việc nhận dữ liệu từ con stm32 thứ nhất sau đó xử lí dữ liệu của các cảm biến sau đó truyền dữ liệu cho Esp32 qua module Sóng LoRa, đồng thời cũng nhận dữ liệu từ con Esp32 sau đó xử lí Data nhận được từ Esp32
  (Request từ người dùng) sau đó truyền dữ liệu đã xử lí cho Stm32 thứ nhất;
  - Esp32 kết nối wifi với web (app) cũng như xử lý việc nhận dữ liệu qua LoRa từ Stm32 thứ 2 sau đó xử lí dự liệu để hiển thị dữ liệu từ các cảm biến lên web (app) song song với đó cũng lấy dữ liệu từ web(app) là những request từ người dùng
  sau đó truyền dữ liệu qua Sóng LoRa cho Stm32 thứ 2. 
