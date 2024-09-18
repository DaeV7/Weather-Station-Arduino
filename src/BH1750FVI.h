class BH1750FVI
{

public:
    uint8_t addressPin;
    typedef enum eDeviceAddress {
      deviceAddress_L = 0x23,
      deviceAddress_H = 0x5C
    } eDeviceAddress_t;
    typedef enum eDeviceMode {
      I2c_DevModeContHighRes     = 0x10,
      I2c_DevModeContHighRes2    = 0x11,
      I2c_DevModeContLowRes      = 0x13,
      I2c_DevModeOneTimeHighRes  = 0x20,
      I2c_DevModeOneTimeHighRes2 = 0x21,
      I2c_DevModeOneTimeLowRes   = 0x23
    } eDeviceMode_t;
    eDeviceMode_t currentMode = I2c_DevModeOneTimeHighRes;
    BH1750FVI(uint8_t addressPin){
        this->addressPin = addressPin;
    };

    void BH1750FVI::begin();
    void BH1750FVI::setMode(eDeviceMode_t mode);
    uint16_t BH1750FVI::getLightIntensity(eDeviceMode_t mode);
    float BH1750FVI::getLightIntensityHighRes();
    float BH1750FVI::getLightIntensityHighRes2();
    float BH1750FVI::getLightIntensityLowRes();

    void BH1750FVI::setAddressL();
    void BH1750FVI::setAddressH();

    private:
    void BH1750FVI::I2CWrite(uint8_t Data);

    typedef enum eDeviceState {
      I2C_deviceStatePowerDown = 0x00,
      I2C_deviceStatePowerUp   = 0x01,
      I2C_deviceStateReset     = 0x07
    } eDeviceState_t;
    eDeviceAddress_t currentDeviceAddress;
};