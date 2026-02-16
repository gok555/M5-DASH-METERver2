# Version 1.2.2 - BLE Fix + Stability
# Ported from M5Core2 U085 CAN Receiver to UIFlow 2.0 (Mini CAN Unit)
import os, sys, io
import M5
from M5 import *
from unit import MiniCANUnit
import time
import machine
import struct
# --- BLE Configuration ---
ble_available = False
try:
    import bluetooth
    from micropython import const
    ble_available = True
    print("BLE Lib OK")
except ImportError:
    print("BLE Library not found")
_BLE_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_BLE_TX_UUID   = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_BLE_RX_UUID   = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
# --- Configuration ---
CAN_TX_PIN = 32
CAN_RX_PIN = 33
CAN_BAUDRATE = 1000000 # 1Mbps
CAN_ID_1 = 144
CAN_ID_2 = 145
# --- Global Variables ---
can_unit = None
ble_uart = None
data_buffer_144 = bytearray(8)
data_buffer_145 = bytearray(8)
last_recv_time = 0
is_online = False
update_counter = 0
last_ble_send_time = 0
# --- UI Widgets ---
lbl_status = None
lbl_id1 = None
lbl_id2 = None
lbl_id3 = None
lbl_id4 = None
lbl_id5 = None
lbl_id6 = None
# -------------------------------------------------------------------------
# BLE Helper Class
# -------------------------------------------------------------------------
class BLEUART:
    def __init__(self, ble, name="M5CAN"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((
            (_BLE_UART_UUID, ((_BLE_TX_UUID, bluetooth.FLAG_NOTIFY), (_BLE_RX_UUID, bluetooth.FLAG_WRITE),)),
        ))
        self._connections = set()
        # advertising_payload: Flags(3) + Name(Length+2) + ServiceUUID(18)
        # Total limit: 31 bytes.
        # "M5CAN" = 5 chars -> 7 bytes.
        # Flags = 3 bytes.
        # ServiceUUID = 18 bytes.
        # Total = 28 bytes. Fits!
        # Original "M5_CAN_MONITOR" = 14 chars -> 16 bytes.
        # Total = 3 + 16 + 18 = 37 bytes -> ERROR -18 (EINVAL)
        self._payload = self.advertising_payload(name=name, services=[_BLE_UART_UUID])
        self._advertise()
    def _irq(self, event, data):
        if event == 1: # Connect
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("BLE Connected")
        elif event == 2: # Disconnect
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            self._advertise()
            print("BLE Disconnected")
    def _advertise(self):
        self._ble.gap_advertise(500000, adv_data=self._payload)
    def send(self, data):
        for conn_handle in self._connections:
            try:
                self._ble.gatts_notify(conn_handle, self._tx_handle, data)
            except: pass
            
    def advertising_payload(self, limited_disc=False, br_edr=False, name=None, services=None, appearance=0):
        # Helper to pack advertising data
        payload = bytearray()
        def _append(adv_type, value):
            nonlocal payload
            payload += struct.pack("BB", len(value) + 1, adv_type) + value
        
        _append(0x01, struct.pack("B", (0x02 if limited_disc else 0x06) + (0x00 if br_edr else 0x04)))
        
        if name: _append(0x09, name)
        
        # If services don't fit, they should go in scan response, but for simplicity here we keep it small.
        if services:
            for uuid in services:
                b_uuid = bytes(uuid)
                if len(b_uuid) == 2:
                    _append(0x03, b_uuid)
                elif len(b_uuid) == 4:
                    _append(0x05, b_uuid)
                elif len(b_uuid) == 16:
                    _append(0x07, b_uuid)
        return payload
# -------------------------------------------------------------------------
# UI & Logic
# -------------------------------------------------------------------------
def setup_ui():
    global lbl_status, lbl_id1, lbl_id2, lbl_id3, lbl_id4, lbl_id5, lbl_id6
    
    Widgets.fillScreen(0x121212)
    # Status Label
    lbl_status = Widgets.Label("INIT...", 95, 7, 1.0, 0x999999, 0x121212, Widgets.FONTS.DejaVu18)
    
    # Headers
    Widgets.Label("IAT", 125, 66, 1.0, 0x0afcf0, 0x121212, Widgets.FONTS.DejaVu18)
    Widgets.Label("RPM", 103, 107, 1.0, 0xf5f1f1, 0x121212, Widgets.FONTS.DejaVu12)
    Widgets.Label("MAP", 87, 189, 1.0, 0x0bf20c, 0x121212, Widgets.FONTS.DejaVu18)
    Widgets.Label("AFR", 253, 68, 1.0, 0xe809fc, 0x121212, Widgets.FONTS.DejaVu12)
    Widgets.Label("VOLT", 248, 126, 1.0, 0xf8d306, 0x121212, Widgets.FONTS.DejaVu18)
    Widgets.Label("EGT", 245, 189, 1.0, 0xfc0707, 0x121212, Widgets.FONTS.DejaVu18)
    
    # Values
    lbl_id1 = Widgets.Label("---", 12, 57, 1.0, 0x07f5ee, 0x121212, Widgets.FONTS.DejaVu24)
    lbl_id2 = Widgets.Label("---", 12, 118, 1.0, 0xf9f9f9, 0x121212, Widgets.FONTS.DejaVu24)
    lbl_id3 = Widgets.Label("---", 12, 180, 1.0, 0x2bf507, 0x121212, Widgets.FONTS.DejaVu24)
    lbl_id4 = Widgets.Label("---", 176, 57, 1.0, 0xea07f5, 0x121212, Widgets.FONTS.DejaVu24)
    lbl_id5 = Widgets.Label("---", 170, 118, 1.0, 0xf5c807, 0x121212, Widgets.FONTS.DejaVu24)
    lbl_id6 = Widgets.Label("---", 170, 180, 1.0, 0xf51907, 0x121212, Widgets.FONTS.DejaVu24)
    
    Widgets.Label("Ver 1.2.2", 5, 220, 1.0, 0xAAAAAA, 0x121212, Widgets.FONTS.DejaVu12)
def get_be_16bit(data, index_1based):
    if data is None: return 0
    pos = (index_1based - 1) * 2
    if pos + 1 >= len(data): return 0
    return (data[pos] << 8) | data[pos+1]
def update_display_logic():
    global update_counter
    # Multiplexing updates
    if update_counter % 2 == 0:
        val = get_be_16bit(data_buffer_144, 1)
        lbl_id1.setText(str(val))
        
    if update_counter % 8 == 2:
        val = get_be_16bit(data_buffer_144, 2)
        lbl_id2.setText(str(val))
        
    if update_counter % 10 == 6:
        val = get_be_16bit(data_buffer_144, 3)
        lbl_id3.setText(str(val - 100))
        
    if update_counter % 6 == 2:
        val = get_be_16bit(data_buffer_145, 1)
        lbl_id4.setText("{:.1f}".format(val * 0.1))
    if update_counter % 6 == 4:
        val = get_be_16bit(data_buffer_145, 2)
        lbl_id5.setText("{:.1f}V".format(val * 0.01))
        
    if update_counter % 10 == 8:
        val = get_be_16bit(data_buffer_145, 3)
        lbl_id6.setText(str(val))
    update_counter += 1
    if update_counter >= 10:
        update_counter = 0
def setup():
    global can_unit, ble_uart
    
    # 1. Reset I2C interference pins (Stability Fix from Ver 1.1.0)
    try:
        p32 = machine.Pin(32, machine.Pin.IN)
        p33 = machine.Pin(33, machine.Pin.IN)
        time.sleep_ms(100)
    except: pass
    M5.begin()
    setup_ui()
    
    # 2. BLE Setup
    print("Initializing BLE...")
    try:
        if ble_available:
            ble = bluetooth.BLE()
            # Try to force deinit in case previous run left it weird
            ble.active(False)
            time.sleep_ms(200)
            
            # Using shorter name to fit in 31-byte Advertising Packet
            ble_uart = BLEUART(ble, name="M5CAN") 
            print("BLE Started! (Name: M5CAN)")
            Widgets.Label("BLE OK", 65, 220, 1.0, 0x00FF00, 0x121212, Widgets.FONTS.DejaVu12)
        else:
            print("BLE Module Missing")
            Widgets.Label("BLE Missing", 65, 220, 1.0, 0xFF0000, 0x121212, Widgets.FONTS.DejaVu12)
    except Exception as e:
        print("BLE Init Failed:", e)
        Widgets.Label("BLE Err: " + str(e), 5, 220, 1.0, 0xFF0000, 0x121212, Widgets.FONTS.DejaVu12)
    # 3. CAN Setup (with Hardware Filter)
    print("Initializing CAN...")
    try:
        can_unit = MiniCANUnit(port=(CAN_RX_PIN, CAN_TX_PIN), mode=MiniCANUnit.NORMAL, baudrate=CAN_BAUDRATE)
        
        # --- Hardware Filter Setup (Stability Fix from Ver 1.1.0) ---
        try:
            can_unit.setfilter(0, MiniCANUnit.MASK16, 0, (0x90, 0x7FE))
            print("CAN Filter Enabled")
        except Exception as e:
            print("Filter Init Error:", e)
        print("CAN Initialized Success")
        lbl_status.setText("OFF")
    except Exception as e:
        lbl_status.setText("CAN Err")
        print("CAN Init Error:", e)
def loop():
    global last_recv_time, is_online, last_ble_send_time
    M5.update()
    
    # CAN Reception
    if can_unit:
        try:
            msg = can_unit.recv(0, timeout=0) 
            if msg:
                can_id = msg[0]
                data = msg[4] 
                
                if can_id == CAN_ID_1:
                    data_buffer_144[:] = data
                elif can_id == CAN_ID_2:
                    data_buffer_145[:] = data
                
                if can_id in (CAN_ID_1, CAN_ID_2):
                    last_recv_time = time.ticks_ms()
                    
                    if not is_online:
                        is_online = True
                        lbl_status.setText("ONLINE")
                        lbl_status.setColor(0x33ff33)
        except Exception:
            pass
    # Status Monitor
    if is_online:
        if time.ticks_diff(time.ticks_ms(), last_recv_time) > 1500:
            is_online = False
            lbl_status.setText("OFF")
            lbl_status.setColor(0x999999)
            
    # Display Update
    update_display_logic()
    
    # BLE Send (Approx 5Hz)
    try:
        if ble_uart and is_online:
            now = time.ticks_ms()
            if time.ticks_diff(now, last_ble_send_time) > 200: 
                last_ble_send_time = now
                
                # Values
                iat = get_be_16bit(data_buffer_144, 1)
                rpm = get_be_16bit(data_buffer_144, 2)
                map_val = get_be_16bit(data_buffer_144, 3) - 100
                afr = get_be_16bit(data_buffer_145, 1) * 0.1
                volt = get_be_16bit(data_buffer_145, 2) * 0.01
                egt = get_be_16bit(data_buffer_145, 3)
                
                # CSV Format: RPM,MAP,AFR,IAT,VOLT,EGT
                # Add \n for line break
                send_str = "{},{},{:.1f},{},{:.1f},{}\n".format(rpm, map_val, afr, iat, volt, egt)
                if ble_available:
                    ble_uart.send(send_str.encode())
    except Exception:
        pass
    time.sleep_ms(10) # Fast loop
if __name__ == '__main__':
    try:
        setup()
        while True:
            loop()
    except (Exception, KeyboardInterrupt) as e:
        # Cleanup
        if can_unit:
            try: can_unit.deinit()
            except: pass
        if ble_available:
            try: 
                import bluetooth
                bluetooth.BLE().active(False)
            except: pass
            
        try:
            from utility import print_error_msg
            print_error_msg(e)
        except ImportError:
            print("please update to latest firmware")
