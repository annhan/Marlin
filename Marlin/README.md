file gcode/quêu.cpp

dong 400 print wait thành w

thay doi file gcode.cpp

dong 956 tu print busy thanh print b + posi 

tHAY DOI FILE MOTION.CPP DONG 212 THEM SERIAL_CHAR(' '); DE TACH REPORT M114

CHINH FILE CONFIGURATION_STORE.CPP CHO REPORT M503 CAC THONG SO SETTING

marrline.cpp them vao
-----------------------

```
void report_pos_step(){
	//stepper.report_positions();
	report_current_position();
}
```
```
M42 Dieu Khien Ngo OUT : 68,69,70,5,6,2,3  VD: M42 P68 S255
M226 Wait input Pin: 54,55,56,57,58, 4,10,53,63,64    VD M226 P55 S0
```   
```
 54 PIN protectiom check lai ->PIN A0 là TEMP_0_PIN
 Đổi TEMP_0_PIN thanh 8 là A8

 Pin 3 trong M42 là Pin default của HEATER_0_PIN
 đổi HEATER_0_PIN  thành D47 là Step7 Not using
 ```