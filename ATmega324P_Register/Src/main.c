#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// ==================== DEFINE ====================
#define TRUE_CARD       1           // Thẻ đúng
#define WRONG_CARD      2           // Thẻ sai
#define CARD_FOUND      0
#define CARD_NOT_FOUND  1
#define ERROR           2
#define MAX_LEN         16

#define Green_Led       PD5         // LED xanh báo thành công
#define Red_Led         PD6         // LED đỏ báo thất bại

#define LCD_SLA         0x4E        // Địa chỉ I2C LCD (0x27 << 1)

#define SPI_DDR         DDRB
#define SPI_MOSI        PB5
#define SPI_SCK         PB7
#define SPI_SS          PB4         // Chân Chip Select cho giao tiếp SPI

#define ENABLE_CHIP()   PORTB &= ~(1<<SPI_SS)     // Kích hoạt thiết bị SPI
#define DISABLE_CHIP()  PORTB |= (1<<SPI_SS)      // Ngắt kết nối thiết bị SPI

// Các lệnh cho giao tiếp TWI/I2C
#define TWI_START       ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN))
#define TWI_STOP        ((1<<TWINT)|(1<<TWSTO)|(1<<TWEN))
#define TWI_READ_ACK    ((1<<TWINT)|(1<<TWEA)|(1<<TWEN))
#define TWI_READ_NACK   ((1<<TWINT)|(1<<TWEN))

// Các thanh ghi và lệnh điều khiển MFRC522
#define SoftReset_CMD     0x0F
#define CommandReg        0x01
#define ComIEnReg         0x02
#define DivIEnReg         0x03
#define FIFOLevelReg      0x0A
#define FIFODataReg       0x09
#define BitFramingReg     0x0D
#define ModeReg           0x11
#define TxASKReg          0x15
#define TxControlReg      0x14
#define ErrorReg          0x06
#define ControlReg        0x0C
#define ComIrqReg         0x04
#define TModeReg          0x2A
#define TPrescalerReg     0x2B
#define TReloadReg_1      0x2C
#define TReloadReg_2      0x2D

#define Transceive_CMD    0x0C
#define MFAuthent_CMD     0x0E
#define Idle_CMD          0x00

#define PICC_REQALL       0x52        // Yêu cầu tất cả các thẻ
#define PICC_ANTICOLL     0x93        // Lệnh chống va chạm

// ==================== BIẾN TOÀN CỤC ====================
uint8_t PASSWORD[5] = {'1', '2', '3', '4', '5'}; // Mật khẩu mặc định
uint8_t KeyMatrix[4][3] = { {'1','2','3'}, {'4','5','6'}, {'7','8','9'}, {'*','0','#'} };
uint8_t ID_Card, byte, index, state = 0;
uint8_t str[MAX_LEN];
char Key[5], key_pressed;

// ==================== HÀM KHAI BÁO ====================
void GPIO_Init();
void RCC_Configuration();
void SPI_Init();
uint8_t SPI_Transmit(uint8_t data);
void mfrc522_init();
void mfrc522_reset();
void mfrc522_write(uint8_t reg, uint8_t data);
uint8_t mfrc522_read(uint8_t reg);
uint8_t mfrc522_to_card(uint8_t cmd, uint8_t *send_data, uint8_t send_data_len, uint8_t *back_data, uint32_t *back_data_len);
uint8_t mfrc522_request(uint8_t req_mode, uint8_t *tag_type);
uint8_t mfrc522_get_card_serial(uint8_t *serial_out);
uint8_t CheckCard(uint8_t *card);
void TWI_Init();
void TWI_Start(uint8_t Addr , uint8_t RW);
void TWI_Stop();
void TWI_Read_block(uint8_t *data, uint8_t len);
void TWI_Write_block(uint8_t Data);
void LCD_Command(unsigned char cmd);
void LCD_Data(unsigned char data);
void LCD_Init();
void LCD_Clear();
void LCD_Char(const char *msg);
void LCD_String(const char *msg,uint8_t len);
void LCD_SetCursor(uint8_t x , uint8_t line);
uint8_t GetKey();
void GetKeypad();
uint8_t checkdata();

// ==================== HÀM MAIN ====================
int main(void)
{
    GPIO_Init();        // Khởi tạo GPIO cho LED và keypad
    TWI_Init();         // Khởi tạo giao tiếp I2C
    SPI_Init();         // Khởi tạo SPI để giao tiếp RFID
    _delay_ms(1000);
    mfrc522_init();     // Khởi tạo MFRC522
    LCD_Init();         // Khởi tạo LCD

    // Hiển thị màn hình khởi động
    LCD_SetCursor(4,0);
    LCD_String("BTL HTN", 7);

    // Bật ngắt cho RFID
    byte = mfrc522_read(ComIEnReg);
    mfrc522_write(ComIEnReg, byte | 0x20);
    byte = mfrc522_read(DivIEnReg);
    mfrc522_write(DivIEnReg, byte | 0x80);
    _delay_ms(2000);

    // Hiển thị hướng dẫn
    LCD_Clear();
    LCD_String("Pass: ", 6);
    LCD_SetCursor(0,1);
    LCD_String("Hoac quet the!", 14);
    LCD_SetCursor(6,0);

    while (1)
    {
        GetKeypad();                        // Lấy dữ liệu từ keypad
        ID_Card = CheckCard(PASSWORD);     // Kiểm tra thẻ (ở đây đang dùng mật khẩu mô phỏng)

        if(ID_Card == TRUE_CARD) state = 1;
        else if(ID_Card == WRONG_CARD) state = 2;

        if(state == 1) {
            LCD_Clear();
            LCD_String("Thanh cong!", 11);
            PORTD |= (1<<Green_Led);
            _delay_ms(3000);
            PORTD &= ~(1<<Green_Led);
        }

        if(state == 2) {
            LCD_Clear();
            LCD_String("That bai!", 9);
            PORTD |= (1<<Red_Led);
            _delay_ms(3000);
            PORTD &= ~(1<<Red_Led);
        }

        if(state) {
            state = 0;
            index = 0;
            LCD_Clear();
            LCD_String("Pass: ", 6);
            LCD_SetCursor(0,1);
            LCD_String("Hoac quet the!", 14);
            LCD_SetCursor(6,0);
        }

        _delay_ms(100);
    }
}

// ==================== CÁC HÀM XỬ LÝ SPI VÀ MODULE RFID ====================
void GPIO_Init() {
    // Cấu hình LED làm output
    DDRD |= (1<<Green_Led)|(1<<Red_Led);
    PORTD &= ~((1<<Green_Led)|(1<<Red_Led));

    // Cấu hình keypad: hàng output, cột input + pull-up
    DDRD |= 0xF8;    // hàng (PD3 -> PD7) output
    DDRD &= ~0x07;   // cột (PD0 -> PD2) input
    PORTD |= 0x07;   // bật pull-up cho cột
}

void SPI_Init() {
    // MOSI, SCK, SS làm output
    SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);
    // Bật SPI, chế độ Master, tốc độ f/16
    SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

uint8_t SPI_Transmit(uint8_t data) {
    SPDR = data;                        // Gửi dữ liệu
    while(!(SPSR & (1<<SPIF)));         // Đợi truyền xong
    return SPDR;                        // Trả về dữ liệu nhận
}

void mfrc522_reset() {
    mfrc522_write(CommandReg, SoftReset_CMD);
    _delay_ms(50);
}

void mfrc522_write(uint8_t reg, uint8_t data) {
    ENABLE_CHIP();
    SPI_Transmit((reg<<1)&0x7E);    // Gửi địa chỉ thanh ghi
    SPI_Transmit(data);             // Gửi dữ liệu
    DISABLE_CHIP();
}

uint8_t mfrc522_read(uint8_t reg) {
    uint8_t data;
    ENABLE_CHIP();
    SPI_Transmit(((reg<<1)&0x7E)|0x80); // Gửi địa chỉ để đọc
    data = SPI_Transmit(0x00);          // Nhận dữ liệu
    DISABLE_CHIP();
    return data;
}

void mfrc522_init() {
    uint8_t byte;
    mfrc522_reset();                        // Reset MFRC522

    // Cấu hình bộ timer nội để tạo tần số thời gian chờ
    mfrc522_write(TModeReg, 0x8D);          // Tự động khởi động timer
    mfrc522_write(TPrescalerReg, 0x3E);     // Đặt Prescaler
    mfrc522_write(TReloadReg_1, 30);        // Reload timer giá trị thấp
    mfrc522_write(TReloadReg_2, 0);         // Reload timer giá trị cao

    mfrc522_write(TxASKReg, 0x40);          // Mở ASK (100% modulation)
    mfrc522_write(ModeReg, 0x3D);           // Đặt chế độ hoạt động

    // Mở anten nếu chưa được bật
    byte = mfrc522_read(TxControlReg);
    if(!(byte & 0x03))
        mfrc522_write(TxControlReg, byte | 0x03);
}

uint8_t mfrc522_to_card(uint8_t cmd, uint8_t *send_data, uint8_t send_data_len,
uint8_t *back_data, uint32_t *back_data_len) 
{ 
	uint8_t status = ERROR; 
	uint8_t irqEn = 0x00; 
	uint8_t waitIRq = 0x00; 
	uint8_t lastBits; 
	uint8_t n; 
	uint8_t tmp; 
	uint32_t i; 
	switch (cmd) 
	{ 
	 case MFAuthent_CMD: 
	 { 
	 irqEn = 0x12; 
	 waitIRq = 0x10; 
	 break; 
	 } 
	 case Transceive_CMD: //Transmit FIFO data
	 { 
	 irqEn = 0x77; 
	 waitIRq = 0x30; 
	 break; 
	 } 
	 default: 
	 break; 
	} 
	n=mfrc522_read(ComIrqReg); 
	mfrc522_write(ComIrqReg,n&(~0x80)); 
	n=mfrc522_read(FIFOLevelReg); 
	mfrc522_write(FIFOLevelReg,n|0x80); 
	mfrc522_write(CommandReg, Idle_CMD); 
	for (i=0; i<send_data_len; i++) 
	{ 
	 mfrc522_write(FIFODataReg, send_data[i]); 
	} 
	mfrc522_write(CommandReg, cmd); 
	if (cmd == Transceive_CMD) 
	{ 
	 n=mfrc522_read(BitFramingReg); 
	 mfrc522_write(BitFramingReg,n|0x80); 
	} 
	i = 2000; 
	do
	{ 
	 n = mfrc522_read(ComIrqReg); 
	 i--; 
	} 
	while ((i!=0) && !(n&0x01) && !(n&waitIRq)); 
	tmp=mfrc522_read(BitFramingReg); 
	mfrc522_write(BitFramingReg,tmp&(~0x80)); 
	if (i != 0) 
	{ 
	 if(!(mfrc522_read(ErrorReg) & 0x1B)) 
	 {
		 status = CARD_FOUND;
		  if (n & irqEn & 0x01) status = CARD_NOT_FOUND; 
		  if (cmd == Transceive_CMD) 
		  { 
		  n = mfrc522_read(FIFOLevelReg); 
		  lastBits = mfrc522_read(ControlReg) & 0x07; 
		  if (lastBits) *back_data_len = (n-1)*8 + lastBits; 
		  else *back_data_len = n*8; 
		  if (n == 0) n = 1; 
		  if (n > MAX_LEN) n = MAX_LEN; 
		  //Reading the received data in FIFO
		  for (i=0; i<n; i++) back_data[i] =
		 mfrc522_read(FIFODataReg); 
		  } 
		  } 
		  else status = ERROR; 
		 } 
		 return status; 
}

uint8_t mfrc522_get_card_serial(uint8_t *serial_out) 
{ 
	uint8_t status; 
	uint8_t i; 
	uint8_t serNumCheck=0; 
	uint32_t unLen; 
	mfrc522_write(BitFramingReg, 0x00); 
	serial_out[0] = PICC_ANTICOLL; 
	serial_out[1] = 0x20; 
	status = mfrc522_to_card(Transceive_CMD, serial_out, 2, serial_out, &unLen); 
	if (status == CARD_FOUND) 
	{ 
	 for (i=0; i<4; i++) serNumCheck ^= serial_out[i]; 
	 if (serNumCheck != serial_out[i]) status = ERROR; 
	} 
	return status; 
}

uint8_t CheckCard(uint8_t *card) 
{ 
	uint8_t byte = 0; 
	byte = mfrc522_request(PICC_REQALL,str); 
	if(byte == CARD_FOUND) 
	{ 
	 byte = mfrc522_get_card_serial(str); 
	 if(byte == CARD_FOUND) 
	 { 
	 for(byte=0;byte<5;byte++) 
	 { 
	 if(*card != str[byte]) return WRONG_CARD; 
	 card++; 
	 } 
	 return TRUE_CARD; 
	 }
	 else return CARD_NOT_FOUND;
	 } 
	 else return CARD_NOT_FOUND; 
}
	 

// ==================== PHẦN XỬ LÝ KEYPAD ====================
uint8_t GetKey() {
    uint8_t KeyTemp = 0, KeyRot = 0x3F;
    PORTD = 0x07;
    _delay_ms(1);
    
    if (PIND == 0x07) return 0xFF;  // Không có phím nào được nhấn

    for (uint8_t i = 0; i < 4; i++) {
        PORTD = KeyRot | 0x07;      // Xoay dòng
        KeyRot = (KeyRot >> 1) | 0x40;
        KeyTemp = PIND & 0x07;

        // Chống rung phím
        for (uint8_t j = 0; j < 100; j++) {
            if (KeyTemp != (PIND & 0x07)) return 0xFF;
            KeyTemp = PIND & 0x07;
        }

        switch (KeyTemp) {
            case 0x03: return KeyMatrix[i][0];
            case 0x05: return KeyMatrix[i][1];
            case 0x06: return KeyMatrix[i][2];
        }
    }

    return 0xFF; // Không xác định
}

void GetKeypad() {
    key_pressed = GetKey();

    // Nếu có phím và chưa nhập đủ 5 ký tự
    if (key_pressed != 0xFF && index < 5) {
        Key[index++] = key_pressed;   // Lưu vào mảng Key
        LCD_Data('*');                // Hiển thị dấu '*'
    }

    // Nếu đã nhập đủ 5 ký tự thì kiểm tra mật khẩu
    if (index == 5) {
        if (checkdata()) state = 1;   // Mật khẩu đúng
        else state = 2;               // Mật khẩu sai
    }
}

uint8_t checkdata() {
    // So sánh chuỗi nhập từ bàn phím với PASSWORD
    for (uint8_t i = 0; i < 5; i++) {
        if (Key[i] != PASSWORD[i]) return 0;
    }
    return 1;
}

// ==================== TWI (I2C) ====================
void TWI_Init() {
    TWSR = 0x00;       // Tốc độ chuẩn
    TWBR = 0x20;       // Tốc độ I2C = 100kHz
}

void TWI_Start(uint8_t Addr, uint8_t RW) {
    TWCR = TWI_START;                       // Gửi điều kiện START
    while (!(TWCR & (1<<TWINT)));           // Chờ truyền xong

    TWDR = (Addr << 1) | RW;                // Gửi địa chỉ + bit đọc/ghi
    TWCR = TWI_READ_NACK;                   // Tiếp tục truyền
    while (!(TWCR & (1<<TWINT)));           // Chờ truyền xong
}

void TWI_Stop() {
    TWCR = TWI_STOP;                        // Gửi điều kiện STOP
    TWI_Init();                             // Khởi tạo lại TWI
}

void TWI_Write_block(uint8_t Data) {
    TWDR = Data;                            // Ghi dữ liệu vào thanh ghi
    TWCR = TWI_READ_NACK;                   // Bắt đầu truyền
    while (!(TWCR & (1<<TWINT)));           // Chờ kết thúc
}

// ==================== LCD ====================
void LCD_Command(unsigned char cmd) {
    TWI_Start(LCD_SLA, 0);                  // Bắt đầu giao tiếp LCD
    TWI_Write_block(0x0C | (0xF0 & cmd));   // Gửi nibble cao + E = 1
    TWI_Write_block(0x08 | (0xF0 & cmd));   // Gửi nibble cao + E = 0
    TWI_Write_block(0x0C | (cmd << 4));     // Gửi nibble thấp + E = 1
    TWI_Write_block(0x08 | (cmd << 4));     // Gửi nibble thấp + E = 0
    _delay_ms(1);
    TWI_Stop();
}

void LCD_Data(unsigned char data) {
    TWI_Start(LCD_SLA, 0);
    TWI_Write_block(0x0D | (0xF0 & data));  // Gửi nibble cao + RS = 1
    TWI_Write_block(0x09 | (0xF0 & data));  // Gửi nibble cao + RS = 1, E = 0
    TWI_Write_block(0x0D | (data << 4));    // Gửi nibble thấp + RS = 1
    TWI_Write_block(0x09 | (data << 4));
    _delay_ms(1);
    TWI_Stop();
}

void LCD_Init() {
    _delay_ms(15);
    LCD_Command(0x02);      // Trở về chế độ 4-bit
    LCD_Command(0x28);      // Giao diện 4-bit, 2 dòng, font 5x8
    LCD_Command(0x0C);      // Bật hiển thị, tắt con trỏ
    LCD_Command(0x06);      // Tự động dịch con trỏ
    LCD_Command(0x01);      // Xóa màn hình
    _delay_ms(5);
}

void LCD_Clear() {
    LCD_Command(0x01);      // Xóa toàn bộ màn hình
    _delay_ms(2);
}

void LCD_Char(const char *msg) {
    while ((*msg) != 0) {
        LCD_Data(*msg);
        msg++;
    }
}

void LCD_String(const char *msg, uint8_t len) {
    while (len--) {
        if (*msg) LCD_Data(*msg++);
        else LCD_Data(' '); // Điền dấu cách nếu thiếu ký tự
    }
}

void LCD_SetCursor(uint8_t x , uint8_t line) {
    LCD_Command(0x80 | (line == 1 ? x + 0x40 : x));  // Đặt con trỏ tại vị trí (x, line)
}