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