<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.5.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="frames" urn="urn:adsk.eagle:library:229">
<description>&lt;b&gt;Frames for Sheet and Layout&lt;/b&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="A4L-LOC" urn="urn:adsk.eagle:symbol:13874/1" library_version="1">
<wire x1="256.54" y1="3.81" x2="256.54" y2="8.89" width="0.1016" layer="94"/>
<wire x1="256.54" y1="8.89" x2="256.54" y2="13.97" width="0.1016" layer="94"/>
<wire x1="256.54" y1="13.97" x2="256.54" y2="19.05" width="0.1016" layer="94"/>
<wire x1="256.54" y1="19.05" x2="256.54" y2="24.13" width="0.1016" layer="94"/>
<wire x1="161.29" y1="3.81" x2="161.29" y2="24.13" width="0.1016" layer="94"/>
<wire x1="161.29" y1="24.13" x2="215.265" y2="24.13" width="0.1016" layer="94"/>
<wire x1="215.265" y1="24.13" x2="256.54" y2="24.13" width="0.1016" layer="94"/>
<wire x1="246.38" y1="3.81" x2="246.38" y2="8.89" width="0.1016" layer="94"/>
<wire x1="246.38" y1="8.89" x2="256.54" y2="8.89" width="0.1016" layer="94"/>
<wire x1="246.38" y1="8.89" x2="215.265" y2="8.89" width="0.1016" layer="94"/>
<wire x1="215.265" y1="8.89" x2="215.265" y2="3.81" width="0.1016" layer="94"/>
<wire x1="215.265" y1="8.89" x2="215.265" y2="13.97" width="0.1016" layer="94"/>
<wire x1="215.265" y1="13.97" x2="256.54" y2="13.97" width="0.1016" layer="94"/>
<wire x1="215.265" y1="13.97" x2="215.265" y2="19.05" width="0.1016" layer="94"/>
<wire x1="215.265" y1="19.05" x2="256.54" y2="19.05" width="0.1016" layer="94"/>
<wire x1="215.265" y1="19.05" x2="215.265" y2="24.13" width="0.1016" layer="94"/>
<text x="217.17" y="15.24" size="2.54" layer="94">&gt;DRAWING_NAME</text>
<text x="217.17" y="10.16" size="2.286" layer="94">&gt;LAST_DATE_TIME</text>
<text x="230.505" y="5.08" size="2.54" layer="94">&gt;SHEET</text>
<text x="216.916" y="4.953" size="2.54" layer="94">Sheet:</text>
<frame x1="0" y1="0" x2="260.35" y2="179.07" columns="6" rows="4" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="A4L-LOC" urn="urn:adsk.eagle:component:13926/1" prefix="FRAME" uservalue="yes" library_version="1">
<description>&lt;b&gt;FRAME&lt;/b&gt;&lt;p&gt;
DIN A4, landscape with location and doc. field</description>
<gates>
<gate name="G$1" symbol="A4L-LOC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="Teensy_3_and_LC_Series_Boards_v1.4">
<packages>
<package name="TEENSY_3.0-3.2&amp;LC_OUTER_ROW">
<pad name="GND" x="-7.62" y="16.51" drill="0.9652"/>
<pad name="0" x="-7.62" y="13.97" drill="0.9652"/>
<pad name="1" x="-7.62" y="11.43" drill="0.9652"/>
<pad name="2" x="-7.62" y="8.89" drill="0.9652"/>
<pad name="3" x="-7.62" y="6.35" drill="0.9652"/>
<pad name="4" x="-7.62" y="3.81" drill="0.9652"/>
<pad name="5" x="-7.62" y="1.27" drill="0.9652"/>
<pad name="6" x="-7.62" y="-1.27" drill="0.9652"/>
<pad name="7" x="-7.62" y="-3.81" drill="0.9652"/>
<pad name="8" x="-7.62" y="-6.35" drill="0.9652"/>
<pad name="9" x="-7.62" y="-8.89" drill="0.9652"/>
<pad name="10" x="-7.62" y="-11.43" drill="0.9652"/>
<pad name="11" x="-7.62" y="-13.97" drill="0.9652"/>
<pad name="12" x="-7.62" y="-16.51" drill="0.9652"/>
<pad name="VBAT" x="-5.08" y="-16.51" drill="0.9652"/>
<pad name="3.3V1" x="-2.54" y="-16.51" drill="0.9652"/>
<pad name="GND1" x="0" y="-16.51" drill="0.9652"/>
<pad name="PGM" x="2.54" y="-16.51" drill="0.9652"/>
<pad name="RESET/DAC" x="5.08" y="-16.51" drill="0.9652"/>
<pad name="13" x="7.62" y="-16.51" drill="0.9652"/>
<pad name="14/A0" x="7.62" y="-13.97" drill="0.9652"/>
<pad name="15/A1" x="7.62" y="-11.43" drill="0.9652"/>
<pad name="16/A2" x="7.62" y="-8.89" drill="0.9652"/>
<pad name="17/A3" x="7.62" y="-6.35" drill="0.9652"/>
<pad name="18/A4" x="7.62" y="-3.81" drill="0.9652"/>
<pad name="19/A5" x="7.62" y="-1.27" drill="0.9652"/>
<pad name="20/A6" x="7.62" y="1.27" drill="0.9652"/>
<pad name="21/A7" x="7.62" y="3.81" drill="0.9652"/>
<pad name="22/A8" x="7.62" y="6.35" drill="0.9652"/>
<pad name="23/A9" x="7.62" y="8.89" drill="0.9652"/>
<pad name="3.3V" x="7.62" y="11.43" drill="0.9652"/>
<pad name="AGND" x="7.62" y="13.97" drill="0.9652"/>
<pad name="VIN" x="7.62" y="16.51" drill="0.9652"/>
<wire x1="-8.89" y1="17.78" x2="8.89" y2="17.78" width="0.127" layer="51"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.127" layer="51"/>
<wire x1="8.89" y1="-17.78" x2="-8.89" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.127" layer="51"/>
<wire x1="-1.27" y1="16.51" x2="1.27" y2="16.51" width="0.2032" layer="21"/>
<wire x1="1.27" y1="16.51" x2="1.27" y2="17.78" width="0.2032" layer="21"/>
<wire x1="1.27" y1="17.78" x2="8.89" y2="17.78" width="0.2032" layer="21"/>
<wire x1="8.89" y1="17.78" x2="8.89" y2="-17.78" width="0.2032" layer="21"/>
<wire x1="8.89" y1="-17.78" x2="-8.89" y2="-17.78" width="0.2032" layer="21"/>
<wire x1="-8.89" y1="-17.78" x2="-8.89" y2="17.78" width="0.2032" layer="21"/>
<wire x1="-8.89" y1="17.78" x2="-1.27" y2="17.78" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="17.78" x2="-1.27" y2="16.51" width="0.2032" layer="21"/>
<text x="-2.54" y="13.97" size="1.27" layer="25" font="vector">&gt;NAME</text>
<text x="-3.81" y="-13.97" size="1.27" layer="27" font="vector">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="TEENSY_3.1-3.2_OUTER_ROW">
<wire x1="-17.78" y1="-35.56" x2="17.78" y2="-35.56" width="0.254" layer="94"/>
<wire x1="17.78" y1="-35.56" x2="17.78" y2="33.02" width="0.254" layer="94"/>
<wire x1="17.78" y1="33.02" x2="-17.78" y2="33.02" width="0.254" layer="94"/>
<wire x1="-17.78" y1="33.02" x2="-17.78" y2="-35.56" width="0.254" layer="94"/>
<pin name="12/MISO" x="-22.86" y="-2.54" visible="pin" length="middle"/>
<pin name="11/MOSI" x="-22.86" y="0" visible="pin" length="middle"/>
<pin name="10/TX2/PWM" x="-22.86" y="2.54" visible="pin" length="middle"/>
<pin name="9/RX2/PWM" x="-22.86" y="5.08" visible="pin" length="middle"/>
<pin name="8/TX3" x="-22.86" y="7.62" visible="pin" length="middle"/>
<pin name="7/RX3" x="-22.86" y="10.16" visible="pin" length="middle"/>
<pin name="6/PWM" x="-22.86" y="12.7" visible="pin" length="middle"/>
<pin name="5/PWM" x="-22.86" y="15.24" visible="pin" length="middle"/>
<pin name="4/CAN-RX-PWM" x="-22.86" y="17.78" visible="pin" length="middle"/>
<pin name="3/CAN-TX/PWM" x="-22.86" y="20.32" visible="pin" length="middle"/>
<pin name="2" x="-22.86" y="22.86" visible="pin" length="middle"/>
<pin name="1/TX1/T" x="-22.86" y="25.4" visible="pin" length="middle"/>
<pin name="0/RX1/T" x="-22.86" y="27.94" visible="pin" length="middle"/>
<pin name="GND" x="22.86" y="20.32" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="AGND" x="22.86" y="7.62" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="3.3V" x="22.86" y="25.4" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="23/A9/T/PWM" x="-22.86" y="-30.48" visible="pin" length="middle"/>
<pin name="22/A8/T/PWM" x="-22.86" y="-27.94" visible="pin" length="middle"/>
<pin name="21/A7/PWM" x="-22.86" y="-25.4" visible="pin" length="middle"/>
<pin name="20/A6/PWM" x="-22.86" y="-22.86" visible="pin" length="middle"/>
<pin name="19/A5/T/SCL0" x="-22.86" y="-20.32" visible="pin" length="middle"/>
<pin name="18/A4/T/SDA0" x="-22.86" y="-17.78" visible="pin" length="middle"/>
<pin name="17/A3/T" x="-22.86" y="-15.24" visible="pin" length="middle"/>
<pin name="16/A2/T" x="-22.86" y="-12.7" visible="pin" length="middle"/>
<pin name="15/A1/T" x="-22.86" y="-10.16" visible="pin" length="middle"/>
<pin name="14/A0" x="-22.86" y="-7.62" visible="pin" length="middle"/>
<pin name="13/SCK/LED" x="-22.86" y="-5.08" visible="pin" length="middle"/>
<pin name="PGM" x="22.86" y="15.24" visible="pin" length="middle" rot="R180"/>
<pin name="VBAT" x="22.86" y="22.86" visible="pin" length="middle" direction="pwr" rot="R180"/>
<text x="-5.588" y="34.29" size="1.27" layer="95" font="vector" ratio="15">&gt;NAME</text>
<text x="-2.794" y="-38.1" size="1.27" layer="96" font="vector" ratio="15">&gt;VALUE</text>
<pin name="A14/DAC" x="22.86" y="2.54" visible="pin" length="middle" rot="R180"/>
<pin name="VIN" x="22.86" y="27.94" visible="pin" length="middle" direction="pwr" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="TEENSY_3.1-3.2_OUTER_ROW">
<description>Footprint for Teensy 3.1 or 3.2 board using all pin connections on the outer perimeter</description>
<gates>
<gate name="G$1" symbol="TEENSY_3.1-3.2_OUTER_ROW" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TEENSY_3.0-3.2&amp;LC_OUTER_ROW">
<connects>
<connect gate="G$1" pin="0/RX1/T" pad="0"/>
<connect gate="G$1" pin="1/TX1/T" pad="1"/>
<connect gate="G$1" pin="10/TX2/PWM" pad="10"/>
<connect gate="G$1" pin="11/MOSI" pad="11"/>
<connect gate="G$1" pin="12/MISO" pad="12"/>
<connect gate="G$1" pin="13/SCK/LED" pad="13"/>
<connect gate="G$1" pin="14/A0" pad="14/A0"/>
<connect gate="G$1" pin="15/A1/T" pad="15/A1"/>
<connect gate="G$1" pin="16/A2/T" pad="16/A2"/>
<connect gate="G$1" pin="17/A3/T" pad="17/A3"/>
<connect gate="G$1" pin="18/A4/T/SDA0" pad="18/A4"/>
<connect gate="G$1" pin="19/A5/T/SCL0" pad="19/A5"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="20/A6/PWM" pad="20/A6"/>
<connect gate="G$1" pin="21/A7/PWM" pad="21/A7"/>
<connect gate="G$1" pin="22/A8/T/PWM" pad="22/A8"/>
<connect gate="G$1" pin="23/A9/T/PWM" pad="23/A9"/>
<connect gate="G$1" pin="3.3V" pad="3.3V 3.3V1"/>
<connect gate="G$1" pin="3/CAN-TX/PWM" pad="3"/>
<connect gate="G$1" pin="4/CAN-RX-PWM" pad="4"/>
<connect gate="G$1" pin="5/PWM" pad="5"/>
<connect gate="G$1" pin="6/PWM" pad="6"/>
<connect gate="G$1" pin="7/RX3" pad="7"/>
<connect gate="G$1" pin="8/TX3" pad="8"/>
<connect gate="G$1" pin="9/RX2/PWM" pad="9"/>
<connect gate="G$1" pin="A14/DAC" pad="RESET/DAC"/>
<connect gate="G$1" pin="AGND" pad="AGND"/>
<connect gate="G$1" pin="GND" pad="GND GND1"/>
<connect gate="G$1" pin="PGM" pad="PGM"/>
<connect gate="G$1" pin="VBAT" pad="VBAT"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="BMP388">
<packages>
<package name="PQFN50P200X200X80-10N">
<wire x1="1.21" y1="-1.21" x2="-1.21" y2="-1.21" width="0.05" layer="39"/>
<wire x1="-1.21" y1="-1.21" x2="-1.21" y2="1.21" width="0.05" layer="39"/>
<wire x1="-1.21" y1="1.21" x2="1.21" y2="1.21" width="0.05" layer="39"/>
<wire x1="1.21" y1="1.21" x2="1.21" y2="-1.21" width="0.05" layer="39"/>
<wire x1="1.03" y1="-1.03" x2="-1.03" y2="-1.03" width="0.127" layer="51"/>
<wire x1="-1.03" y1="-1.03" x2="-1.03" y2="1.03" width="0.127" layer="51"/>
<wire x1="-1.03" y1="1.03" x2="1.03" y2="1.03" width="0.127" layer="51"/>
<wire x1="1.03" y1="1.03" x2="1.03" y2="-1.03" width="0.127" layer="51"/>
<wire x1="-0.85" y1="-1.03" x2="-1.03" y2="-1.03" width="0.127" layer="21"/>
<wire x1="-1.03" y1="-1.03" x2="-1.03" y2="-0.63" width="0.127" layer="21"/>
<wire x1="-1.03" y1="0.63" x2="-1.03" y2="1.03" width="0.127" layer="21"/>
<wire x1="-1.03" y1="1.03" x2="-0.85" y2="1.03" width="0.127" layer="21"/>
<wire x1="0.85" y1="1.03" x2="1.03" y2="1.03" width="0.127" layer="21"/>
<wire x1="1.03" y1="1.03" x2="1.03" y2="0.63" width="0.127" layer="21"/>
<wire x1="0.85" y1="-1.03" x2="1.03" y2="-1.03" width="0.127" layer="21"/>
<wire x1="1.03" y1="-1.03" x2="1.03" y2="-0.63" width="0.127" layer="21"/>
<circle x="-1.5" y="0.3" radius="0.1" width="0.2" layer="51"/>
<circle x="-1.5" y="0.3" radius="0.1" width="0.2" layer="21"/>
<text x="-1.301759375" y="1.4019" size="0.40695" layer="25">&gt;NAME</text>
<text x="-1.20183125" y="-1.702590625" size="0.40701875" layer="27">&gt;VALUE</text>
<smd name="1" x="-0.765" y="0.25" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="2" x="-0.765" y="-0.25" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="3" x="-0.5" y="-0.765" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="4" x="0" y="-0.765" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="5" x="0.5" y="-0.765" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="6" x="0.765" y="-0.25" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="7" x="0.765" y="0.25" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="8" x="0.5" y="0.765" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="9" x="0" y="0.765" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
<smd name="10" x="-0.5" y="0.765" dx="0.38" dy="0.35" layer="1" roundness="9" rot="R90"/>
</package>
</packages>
<symbols>
<symbol name="BMP388">
<wire x1="7.62" y1="-12.7" x2="-7.62" y2="-12.7" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-12.7" x2="-7.62" y2="15.24" width="0.254" layer="94"/>
<wire x1="-7.62" y1="15.24" x2="7.62" y2="15.24" width="0.254" layer="94"/>
<wire x1="7.62" y1="15.24" x2="7.62" y2="-12.7" width="0.254" layer="94"/>
<text x="-7.628109375" y="15.2562" size="1.779890625" layer="95" ratio="10">&gt;NAME</text>
<text x="-7.6268" y="-15.2536" size="1.779590625" layer="96" ratio="10">&gt;VALUE</text>
<pin name="VDDIO" x="12.7" y="10.16" length="middle" direction="pwr" rot="R180"/>
<pin name="SCK" x="-12.7" y="2.54" length="middle" direction="in" function="clk"/>
<pin name="VSS" x="12.7" y="-10.16" length="middle" direction="pwr" rot="R180"/>
<pin name="SDI" x="-12.7" y="-2.54" length="middle"/>
<pin name="SDO" x="-12.7" y="-5.08" length="middle"/>
<pin name="CSB" x="-12.7" y="5.08" length="middle" direction="in"/>
<pin name="INT" x="12.7" y="5.08" length="middle" direction="out" rot="R180"/>
<pin name="VDD" x="12.7" y="12.7" length="middle" direction="pwr" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="BMP388" prefix="U">
<description>P-Sensor SPI I2C 1, 25 Bar SMD</description>
<gates>
<gate name="G$1" symbol="BMP388" x="0" y="0"/>
</gates>
<devices>
<device name="" package="PQFN50P200X200X80-10N">
<connects>
<connect gate="G$1" pin="CSB" pad="6"/>
<connect gate="G$1" pin="INT" pad="7"/>
<connect gate="G$1" pin="SCK" pad="2"/>
<connect gate="G$1" pin="SDI" pad="4"/>
<connect gate="G$1" pin="SDO" pad="5"/>
<connect gate="G$1" pin="VDD" pad="10"/>
<connect gate="G$1" pin="VDDIO" pad="1"/>
<connect gate="G$1" pin="VSS" pad="3 8 9"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value=" P-Sensor SPI I2C 1, 25 Bar SMD "/>
<attribute name="DIGI-KEY_PART_NUMBER" value="828-1079-1-ND"/>
<attribute name="DIGI-KEY_PURCHASE_URL" value="https://www.digikey.ca/product-detail/en/bosch-sensortec/BMP388/828-1079-1-ND/8322640?utm_source=snapeda&amp;utm_medium=aggregator&amp;utm_campaign=symbol"/>
<attribute name="MF" value="Bosch Sensortec"/>
<attribute name="MP" value="BMP388"/>
<attribute name="PACKAGE" value="WFLGA-10 Bosch Tools"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="LSM9DS1TR">
<packages>
<package name="PQFN43P300X350X102-24N">
<wire x1="-1.83" y1="-1.58" x2="-1.83" y2="1.58" width="0.127" layer="51"/>
<wire x1="-1.83" y1="1.58" x2="1.83" y2="1.58" width="0.127" layer="51"/>
<wire x1="1.83" y1="1.58" x2="1.83" y2="-1.58" width="0.127" layer="51"/>
<wire x1="1.83" y1="-1.58" x2="-1.83" y2="-1.58" width="0.127" layer="51"/>
<wire x1="-1.99" y1="-1.74" x2="-1.99" y2="1.74" width="0.05" layer="39"/>
<wire x1="-1.99" y1="1.74" x2="1.99" y2="1.74" width="0.05" layer="39"/>
<wire x1="1.99" y1="1.74" x2="1.99" y2="-1.74" width="0.05" layer="39"/>
<wire x1="1.99" y1="-1.74" x2="-1.99" y2="-1.74" width="0.05" layer="39"/>
<circle x="-2.4" y="1.2" radius="0.1" width="0.2" layer="21"/>
<circle x="-2.4" y="1.2" radius="0.1" width="0.2" layer="51"/>
<text x="-2.00321875" y="1.903059375" size="0.40705625" layer="25">&gt;NAME</text>
<text x="-2.0016" y="-2.201759375" size="0.406725" layer="27">&gt;VALUE</text>
<wire x1="-1.86" y1="1.58" x2="-1.86" y2="0.94" width="0.127" layer="21"/>
<wire x1="1.86" y1="1.58" x2="1.86" y2="0.94" width="0.127" layer="21"/>
<wire x1="-1.86" y1="-1.58" x2="-1.86" y2="-0.94" width="0.127" layer="21"/>
<wire x1="1.86" y1="-1.58" x2="1.86" y2="-0.94" width="0.127" layer="21"/>
<smd name="1" x="-1.505" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
<smd name="2" x="-1.47" y="0.645" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R180"/>
<smd name="3" x="-1.47" y="0.215" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R180"/>
<smd name="4" x="-1.47" y="-0.215" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R180"/>
<smd name="5" x="-1.47" y="-0.645" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R180"/>
<smd name="6" x="-1.505" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="7" x="-1.075" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="8" x="-0.645" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="9" x="-0.215" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="10" x="0.215" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="11" x="0.645" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="12" x="1.075" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="13" x="1.505" y="-1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R270"/>
<smd name="14" x="1.47" y="-0.645" dx="0.53" dy="0.28" layer="1" roundness="7"/>
<smd name="15" x="1.47" y="-0.215" dx="0.53" dy="0.28" layer="1" roundness="7"/>
<smd name="16" x="1.47" y="0.215" dx="0.53" dy="0.28" layer="1" roundness="7"/>
<smd name="17" x="1.47" y="0.645" dx="0.53" dy="0.28" layer="1" roundness="7"/>
<smd name="18" x="1.505" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
<smd name="19" x="1.075" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
<smd name="20" x="0.645" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
<smd name="21" x="0.215" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
<smd name="22" x="-0.215" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
<smd name="23" x="-0.645" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
<smd name="24" x="-1.075" y="1.22" dx="0.53" dy="0.28" layer="1" roundness="7" rot="R90"/>
</package>
</packages>
<symbols>
<symbol name="LSM9DS1TR">
<wire x1="12.7" y1="-22.86" x2="-12.7" y2="-22.86" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-22.86" x2="-12.7" y2="25.4" width="0.254" layer="94"/>
<wire x1="-12.7" y1="25.4" x2="12.7" y2="25.4" width="0.254" layer="94"/>
<wire x1="12.7" y1="25.4" x2="12.7" y2="-22.86" width="0.254" layer="94"/>
<text x="-12.7213" y="26.0786" size="1.78098125" layer="95">&gt;NAME</text>
<text x="-12.7214" y="-25.4427" size="1.780990625" layer="96">&gt;VALUE</text>
<pin name="VDDIO" x="17.78" y="20.32" length="middle" direction="pwr" rot="R180"/>
<pin name="SCL/SPC" x="-17.78" y="15.24" length="middle" direction="in" function="clk"/>
<pin name="SDA/SDI/SDO" x="-17.78" y="12.7" length="middle"/>
<pin name="SDO_A/G" x="17.78" y="7.62" length="middle" direction="out" rot="R180"/>
<pin name="SDO_M" x="17.78" y="-2.54" length="middle" direction="out" rot="R180"/>
<pin name="CS_A/G" x="-17.78" y="7.62" length="middle" direction="in"/>
<pin name="CS_M" x="-17.78" y="-5.08" length="middle" direction="in"/>
<pin name="DRDY_M" x="17.78" y="-7.62" length="middle" direction="out" rot="R180"/>
<pin name="INT_M" x="17.78" y="-5.08" length="middle" direction="out" rot="R180"/>
<pin name="INT1_A/G" x="17.78" y="5.08" length="middle" direction="out" rot="R180"/>
<pin name="INT2_A/G" x="17.78" y="2.54" length="middle" direction="out" rot="R180"/>
<pin name="DEN_A/G" x="-17.78" y="5.08" length="middle" direction="in"/>
<pin name="GND" x="17.78" y="-20.32" length="middle" direction="pwr" rot="R180"/>
<pin name="CAP" x="17.78" y="-15.24" length="middle" direction="pas" rot="R180"/>
<pin name="VDD" x="17.78" y="22.86" length="middle" direction="pwr" rot="R180"/>
<pin name="C1" x="17.78" y="-12.7" length="middle" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="LSM9DS1TR" prefix="U">
<description>The LSM9DS1 is a system-in-package featuring a
3D digital linear acceleration sensor, a 3D digital
angular rate sensor, and a 3D digital magnetic
sensor.</description>
<gates>
<gate name="G$1" symbol="LSM9DS1TR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="PQFN43P300X350X102-24N">
<connects>
<connect gate="G$1" pin="C1" pad="24"/>
<connect gate="G$1" pin="CAP" pad="21"/>
<connect gate="G$1" pin="CS_A/G" pad="7"/>
<connect gate="G$1" pin="CS_M" pad="8"/>
<connect gate="G$1" pin="DEN_A/G" pad="13"/>
<connect gate="G$1" pin="DRDY_M" pad="9"/>
<connect gate="G$1" pin="GND" pad="19 20"/>
<connect gate="G$1" pin="INT1_A/G" pad="11"/>
<connect gate="G$1" pin="INT2_A/G" pad="12"/>
<connect gate="G$1" pin="INT_M" pad="10"/>
<connect gate="G$1" pin="SCL/SPC" pad="2"/>
<connect gate="G$1" pin="SDA/SDI/SDO" pad="4"/>
<connect gate="G$1" pin="SDO_A/G" pad="5"/>
<connect gate="G$1" pin="SDO_M" pad="6"/>
<connect gate="G$1" pin="VDD" pad="22 23"/>
<connect gate="G$1" pin="VDDIO" pad="1 3"/>
</connects>
<technologies>
<technology name="">
<attribute name="DESCRIPTION" value=" LSM9DS1 Series ±2/±4/±8/±16 g 3D Digital Linear Acceleration Sensor - LGA-24 "/>
<attribute name="DIGI-KEY_PART_NUMBER" value="497-14946-1-ND"/>
<attribute name="DIGI-KEY_PURCHASE_URL" value="https://www.digikey.ca/product-detail/en/stmicroelectronics/LSM9DS1TR/497-14946-1-ND/4988079?utm_source=snapeda&amp;utm_medium=aggregator&amp;utm_campaign=symbol"/>
<attribute name="MF" value="STMicroelectronics"/>
<attribute name="MP" value="LSM9DS1TR"/>
<attribute name="PACKAGE" value="TFLGA-24 STMicroelectronics"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="FRAME1" library="frames" library_urn="urn:adsk.eagle:library:229" deviceset="A4L-LOC" device=""/>
<part name="U$1" library="Teensy_3_and_LC_Series_Boards_v1.4" deviceset="TEENSY_3.1-3.2_OUTER_ROW" device=""/>
<part name="U1" library="BMP388" deviceset="BMP388" device=""/>
<part name="U2" library="LSM9DS1TR" deviceset="LSM9DS1TR" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="FRAME1" gate="G$1" x="0" y="0" smashed="yes">
<attribute name="DRAWING_NAME" x="217.17" y="15.24" size="2.54" layer="94"/>
<attribute name="LAST_DATE_TIME" x="217.17" y="10.16" size="2.286" layer="94"/>
<attribute name="SHEET" x="230.505" y="5.08" size="2.54" layer="94"/>
</instance>
<instance part="U$1" gate="G$1" x="134.62" y="101.6" smashed="yes">
<attribute name="NAME" x="129.032" y="135.89" size="1.27" layer="95" font="vector" ratio="15"/>
<attribute name="VALUE" x="131.826" y="63.5" size="1.27" layer="96" font="vector" ratio="15"/>
</instance>
<instance part="U1" gate="G$1" x="40.64" y="149.86" smashed="yes">
<attribute name="NAME" x="33.011890625" y="165.1162" size="1.779890625" layer="95" ratio="10"/>
<attribute name="VALUE" x="33.0132" y="134.6064" size="1.779590625" layer="96" ratio="10"/>
</instance>
<instance part="U2" gate="G$1" x="30.48" y="93.98" smashed="yes">
<attribute name="NAME" x="17.7587" y="120.0586" size="1.78098125" layer="95"/>
<attribute name="VALUE" x="17.7586" y="68.5373" size="1.780990625" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
