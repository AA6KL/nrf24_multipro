/* 
	Editor: http://www.visualmicro.com
			visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
			the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
			all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
			note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: SparkFun Pro Micro w/ ATmega32U4 (5V, 16 MHz), Platform=avr, Package=SparkFun
*/

#define __AVR_ATmega32u4__
#define __AVR_ATmega32U4__
#define ARDUINO 10802
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define F_CPU 16000000L
#define ARDUINO 10802
#define ARDUINO_AVR_PROMICRO
#define ARDUINO_ARCH_AVR
#define USB_VID 0x1b4f
#define USB_PID 0x9206
//
//
void set_txid(bool renew);
void selectProtocol();
void init_protocol();
void update_ppm();
void ISR_ppm();
void print_ppm();
uint32_t process_Bayang();
void Bayang_init();
void Bayang_bind();
void Bayang_send_packet(u8 bind);
static uint8_t Bayang_checksum();
static uint8_t Bayang_check_rx();
uint32_t process_CG023();
void CG023_init();
void CG023_bind();
void CG023_WritePacket(uint8_t init);
uint32_t process_CX10();
void CX10_init();
void CX10_bind();
void CX10_Write_Packet(uint8_t init);
uint32_t process_FQ777124();
void FQ777124_init();
void FQ777124_bind();
uint16_t nrf_crc(uint8_t data[], uint8_t len, uint16_t crc);
void ssv_pack_dpl(uint8_t addr[], uint8_t pid, uint8_t* len, uint8_t* payload, uint8_t* packed_payload);
void FQ777124_send_packet(u8 bind);
void H7_initTXID();
void H7_init();
void H7_bind();
uint8_t H7_calcChecksum();
void H7_WritePacket();
uint32_t process_H7();
uint32_t process_H8_3D();
void H8_3D_bind();
uint8_t  H8_3D_checksum();
void H8_3D_send_packet(uint8_t  bind);
void H8_3D_init();
uint32_t process_HiSky();
void HiSky_init();
void HiSky_calc_fh_channels();
void HiSky_build_binding_packet(void);
void HiSky_build_ch_data();
void kn_start_tx(u8 bind_yes);
u32 process_KN();
void kn_init(u8 tx_addr[], u8 hopping_ch[]);
void kn_calculate_freqency_hopping_channels(u32 seed, u8 hopping_channels[], u8 tx_addr[]);
void kn_calculate_tx_addr(u8 tx_addr[]);
void kn_bind_init(u8 tx_addr[], u8 hopping_ch[], u8 bind_packet[]);
void kn_send_packet(u8 packet[], int32_t rf_ch);
void kn_send_init(u8 tx_addr[], u8 packet[]);
void kn_update_packet_send_count(u8 packet[], int32_t packet_sent, int32_t rf_ch_idx );
void kn_update_packet_control_data(u8 packet[], int32_t packet_count, int32_t rf_ch_idx);
void kn_read_controls(u16* throttle, u16* aileron, u16* elevator, u16* rudder, u8* flags);
u16 kn_convert_channel(u8 num);
u8 mjx_checksum();
u8 mjx_convert_channel(u8 num);
u8 mjx_pan_tilt_value();
void mjx_send_packet(u8 bind);
uint32_t process_MJX();
void initialize_mjx_txid();
void MJX_init();
void mjx_init2();
void MJX_bind();
uint8_t SYMAX_checksum(uint8_t *data);
void SYMAX_build_packet_x5c(uint8_t bind);
void SYMAX_build_packet(uint8_t bind);
void SYMAX_send_packet(uint8_t bind);
void init_Symax();
void symax_init1();
void symax_set_channels(uint8_t address);
void symax_init2();
uint32_t process_SymaX();
void Symax_init();
void V2x2_init();
void V2x2_bind();
uint32_t process_V2x2();
void V2x2_set_tx_id();
void V2x2_add_pkt_checksum();
void V2x2_set_flags(uint16_t* flags);
uint8_t V2x2_convert_channel(uint8_t num);
void V2x2_send_packet(uint8_t bind);
uint8_t bit_reverse(uint8_t b_in);
uint16_t crc16_update(uint16_t crc, unsigned char a);
void XN297_SetTXAddr(const uint8_t* addr, uint8_t len);
void XN297_SetRXAddr(const uint8_t* addr, uint8_t len);
void XN297_Configure(uint8_t flags);
uint8_t XN297_WritePayload(uint8_t* msg, uint8_t len);
uint8_t XN297_ReadPayload(uint8_t* msg, uint8_t len);
uint8_t YD717_packet_ack();
void YD717_send_packet(uint8_t bind);
void YD717_initialize();
void YD717_init1();
void YD717_init2();
uint32_t process_YD717();
void set_rx_tx_addr(uint32_t id);
void initialize_rx_tx_addr();
void YD717_init();
void frskyInit();
void smartportSend(uint8_t *p);
void smartportIdle();
void smartportSendFrame();
void frskyUpdate();
uint8_t NRF24L01_WriteReg(uint8_t address, uint8_t data);
void NRF24L01_WriteRegisterMulti(uint8_t address, const uint8_t data[], uint8_t len);
void NRF24L01_Initialize();
uint8_t NRF24L01_FlushTx();
uint8_t NRF24L01_FlushRx();
uint8_t Strobe(uint8_t state);
uint8_t NRF24L01_WritePayload(uint8_t *data, uint8_t length);
uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length);
uint8_t NRF24L01_ReadReg(uint8_t reg);
uint8_t NRF24L01_Activate(uint8_t code);
void NRF24L01_SetTxRxMode(enum TXRX_State mode);
uint8_t NRF24L01_Reset();
uint8_t NRF24L01_SetPower(enum TX_Power power);
uint8_t NRF24L01_SetBitrate(uint8_t bitrate);
void Read_Packet(uint8_t *data, uint8_t length);
uint8_t spi_write(uint8_t command);
void spi_write_address(uint8_t address, uint8_t data);
uint8_t spi_read();
uint8_t spi_read_address(uint8_t address);

#include "pins_arduino.h" 
#include "arduino.h"
#include "nRF24_multipro.ino"
#include "Bayang.ino"
#include "CG023.ino"
#include "CX10_GreenBlue.ino"
#include "FQ777-124.ino"
#include "H7.ino"
#include "H8_3D.ino"
#include "HiSky.ino"
#include "KN.ino"
#include "MJX.ino"
#include "SymaX.ino"
#include "V2x2.ino"
#include "XN297_emu.ino"
#include "YD717.ino"
#include "frsky_telemetry.ino"
#include "nRF24L01.ino"
#include "softSPI.ino"
