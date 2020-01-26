package frc.vitruvianlib.I2C;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class LCDDisplay extends I2C {
    byte LCD_CLEARDISPLAY = 0x01;
    byte LCD_RETURNHOME = 0x02;
    byte LCD_ENTRYMODESET = 0x04;
    byte LCD_DISPLAYCONTROL = 0x08;
    byte LCD_CURSORSHIFT = 0x10;
    byte LCD_FUNCTIONSET = 0x20;
    byte LCD_SETCGRAMADDR = 0x40;
    int  LCD_SETDDRAMADDR = 0x80;
    
    // flags for display entry mode
    byte LCD_ENTRYRIGHT = 0x00;
    byte LCD_ENTRYLEFT = 0x02;
    byte LCD_ENTRYSHIFTINCREMENT = 0x01;
    byte LCD_ENTRYSHIFTDECREMENT = 0x00;
    
    // flags for display on/off control
    byte LCD_DISPLAYON = 0x04;
    byte LCD_DISPLAYOFF = 0x00;
    byte LCD_CURSORON = 0x02;
    byte LCD_CURSOROFF = 0x00;
    byte LCD_BLINKON = 0x01;
    byte LCD_BLINKOFF = 0x00;
    
    // flags for display/cursor shift
    byte LCD_DISPLAYMOVE = 0x08;
    byte LCD_CURSORMOVE = 0x00;
    byte LCD_MOVERIGHT = 0x04;
    byte LCD_MOVELEFT = 0x00;
    
    // flags for function set
    byte LCD_8BITMODE = 0x10;
    byte LCD_4BITMODE = 0x00;
    byte LCD_2LINE = 0x08;
    byte LCD_1LINE = 0x00;
    byte LCD_5x10DOTS = 0x04;
    byte LCD_5x8DOTS = 0x00;
    
    // flags for backlight control
    byte LCD_BACKLIGHT = 0x08;
    byte LCD_NOBACKLIGHT = 0x00;
    
    byte En = 0x4;  // Enable bit
    byte Rw = 0x2; // Read/Write bit
    byte Rs = 0x1;  // Register select bit

    private Port port;
    private int _Addr;
    private int _cols = 20;
    private int _rows = 4;
    private int _displayfunction;
    private int _displaycontrol;
    private int _displaymode;
    private int _numlines;
    private int _backlightval;

    public LCDDisplay(Port port, int address) {
        super(port, address);
        this.port = port;
        this._Addr = address;

        _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    }

    public void setBacklight(boolean mode) {
        _backlightval = mode ? LCD_BACKLIGHT : LCD_NOBACKLIGHT;
    }

    public void clear(){
        command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
        Timer.delay(2.0e-5);  // this command takes a long time!
    }

    public void home(){
        command(LCD_RETURNHOME);  // set cursor position to zero
        Timer.delay(2.0e-5);  // this command takes a long time!
    }

    public void setCursor(int col, int row){
        int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
        if ( row > (_numlines-1) ) {
            row = _numlines-1;    // we count rows starting w/0
        }
        command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
    }

    // Turn the display on/off (quickly)
    public void noDisplay() {
        _displaycontrol &= ~LCD_DISPLAYON;
        command(LCD_DISPLAYCONTROL | _displaycontrol);
    }
    public void display() {
        _displaycontrol |= LCD_DISPLAYON;
        command(LCD_DISPLAYCONTROL | _displaycontrol);
    }

    // Turns the underline cursor on/off
    public void noCursor() {
        _displaycontrol &= ~LCD_CURSORON;
        command(LCD_DISPLAYCONTROL | _displaycontrol);
    }
    public void cursor() {
        _displaycontrol |= LCD_CURSORON;
        command(LCD_DISPLAYCONTROL | _displaycontrol);
    }

    // Turn on and off the blinking cursor
    public void noBlink() {
        _displaycontrol &= ~LCD_BLINKON;
        command(LCD_DISPLAYCONTROL | _displaycontrol);
    }
    public void blink() {
        _displaycontrol |= LCD_BLINKON;
        command(LCD_DISPLAYCONTROL | _displaycontrol);
    }

    // These commands scroll the display without changing the RAM
    public void scrollDisplayLeft() {
        command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
    }
    public void scrollDisplayRight() {
        command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
    }

    // This is for text that flows Left to Right
    public void leftToRight() {
        _displaymode |= LCD_ENTRYLEFT;
        command(LCD_ENTRYMODESET | _displaymode);
    }

    // This is for text that flows Right to Left
    public void rightToLeft() {
        _displaymode &= ~LCD_ENTRYLEFT;
        command(LCD_ENTRYMODESET | _displaymode);
    }

    // This will 'right justify' text from the cursor
    public void autoscroll() {
        _displaymode |= LCD_ENTRYSHIFTINCREMENT;
        command(LCD_ENTRYMODESET | _displaymode);
    }

    // This will 'left justify' text from the cursor
    public void noAutoscroll() {
        _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
        command(LCD_ENTRYMODESET | _displaymode);
    }

    // Allows us to fill the first 8 CGRAM locations
    // with custom characters
    public void createChar(int location, int[] charmap) {
        location &= 0x7; // we only have 8 locations 0-7
        command(LCD_SETCGRAMADDR | (location << 3));
        for (int i=0; i<8; i++) {
            write(_Addr, charmap[i]);
        }
    }

    public void print(String message){
        char[] charArray = message.toCharArray();
        byte[] data = new byte[charArray.length];
        for(int i =0; i<charArray.length; i++)
            data[i] = (byte) charArray[i];

        transaction(data, data.length, null, 0);
    }

    private void command(int value) {
        send(value, 0);
    }

    private void send(int value, int mode) {
        int highnib = value & 0xf0;
        int lownib = (value << 4) & 0xf0;
        write4bits((highnib)|mode);
        write4bits((lownib)|mode);
    }
    private void write4bits(int value) {
        expanderWrite(value);
        pulseEnable(value);
    }

    private void expanderWrite(int _data){
        write(_Addr, ((int)(_data) | _backlightval));
    }

    private void pulseEnable(int _data){
        expanderWrite(_data | En);	// En high
        Timer.delay(4.5e-7);		// enable pulse must be >450ns 1.0e-5

        expanderWrite(_data & ~En);	// En low
        Timer.delay(3.7e-5);		// commands need > 37us to settle 5.0e-5
    }
}
