## Chỉnh sữa Code

1. Add G28 Scara
- (PULL)[https://github.com/MarlinFirmware/Marlin/pull/17274]
2. Chỉnh sữa lệnh debug
- Khi Busy thì MArlin sẽ gửi thêm tọa độ X,Y,Z để  giúp ta hiển thị HMI
3. Chỉnh thời gian send information.
- Wait -> 600ms
- Busy ->300ms
- Chỉnh M503 Report các thông số cân thiết
4. Chỉnh JOG cho scara
- 
5. 
file gcode/quêu.cpp

dong 400 print wait thành w

thay doi file gcode.cpp

dong 956 tu print busy thanh print b + posi 

tHAY DOI FILE MOTION.CPP DONG 212 THEM SERIAL_CHAR(' '); DE TACH REPORT M114

CHINH FILE CONFIGURATION_STORE.CPP CHO REPORT M503 CAC THONG SO SETTING

marrline.cpp them vao
-----------------------

```C++
void report_pos_step(){
	//stepper.report_positions();
	report_current_position();
}
```
## PIN IN OUT

1. PIN Number.
```
M42 Dieu Khien Ngo OUT : 68,69,70,7,8,9,2  VD: M42 P68 S255
M226 Wait input Pin: 54,55,56,57,58, 4,10,53,63,64    VD M226 P55 S0
```
2. Pin Label   
```
- PWM PED label -> PIN 2
- E2 Label -> PIN 7
- E1 label -> PIN 8
- E3 label -> PIN 9

- E0 label -> PIN 3 ->using sprindle ENB
- FAN 2 label -> PIN 5 ->using sprindle DIR
- FAN 1 Label -> PIN 6 ->using sprindle PWM
```

3. PIN Modify
```
 54 PIN protectiom check lai ->PIN A0 là TEMP_0_PIN
 Đổi TEMP_0_PIN thanh 8 là A8

 Pin 3 trong M42 là Pin default của HEATER_0_PIN
 đổi HEATER_0_PIN  thành D47 là Step7 Not using
 ```

 ```
 PIN 66,67 trong OUTPUT là PIn DAC0,1 hiện chưa hoạt động được.
 ```