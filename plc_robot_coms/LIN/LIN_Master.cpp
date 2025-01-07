//
//                     ------------------ LIN protocol master on usb ------------------------------
// 
#include <thread>
#include <stdint>
#include <vector>
#include "BoostSerial.h"        // get from here https://github.com/Tai-Min/Boost-Serial-Port?ysclid=m5l7x6yy98688984556

#define DEFAULT_TIMEOUT 15      // port time out

// define the boost serial device which we will receive from the LIN master
BoostSerial s;

#define LIN_TX_ID           0x3Au
#define LIN_RX_ID           0x3Bu
#define LIN_SYNC_BYTE       0x55u
#define LIN_SYNC_CLSE       0x56u

#define LIN_DATA_BYTES_NUM     9u
uint8_t linMasterData[LIN_DATA_BYTES_NUM];
uint8_t linSlaveData[LIN_DATA_BYTES_NUM];

uint16_t linMasterTxCnt = 0u;
uint16_t linMasterRxCnt = 0u;

uint8_t LIN_CalcCheckSum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    
    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
  
    while(sum > 0xFF)
    {
        sum -= 0xFF;
    }
  
    sum = 0xFF - sum;
  
    return (uint8_t)sum;
}

// sequence of messages for LIN protocol
typedef enum
{
    LIN_SENDING_BREAK,
    LIN_SENDING_RX,
    LIN_RECEIVING_DATA,
    LIN_SENDING_TX,
    LIN_SENDING_DATA,
    LIN_CLOSE_PORT,
} LIN_MState;

// initialise the state engine
LIN_MState masterState = LIN_SENDING_BREAK;

// initialise the receive byte
uint8_t rxByte = 0;

// perfroms the LIN Master actions
//
void LIN_Master()
{
        uint8_t checkSum = 0;
        switch(masterState)
        {
            case LIN_SENDING_BREAK:
            {
				if (s.write(LIN_SYNC_BYTE) == 1) {
                    masterState = LIN_SENDING_RX;
				}
            }
            break;
			
            case LIN_SENDING_RX:
			{ 
				if (s.write(LIN_RX_ID) == 1) {
                    std::vector<uint8_t> v = s.readBytes(LIN_DATA_BYTES_NUM);
					for (uint8_t i = 0; i < LIN_DATA_BYTES_NUM - 1; i++)
                    {
                        linMasterData[i] = v[i];                                                  // v.at(i)
                    }
					masterState = LIN_RECEIVING_DATA;
				}
            }
            break;

            case LIN_RECEIVING_DATA:
			{
                checkSum = LIN_CalcCheckSum(linMasterData, LIN_DATA_BYTES_NUM - 1);
        
                if (linMasterData[LIN_DATA_BYTES_NUM - 1] == checkSum)
                {
                    linMasterRxCnt++;
				    std::cout << "slave received packet total packets= " << linMasterRxCnt << std::endl;
                } else {
				    std::cout << "slave received packet with invalid checksum" << std::endl;			
			    }
        
                masterState = LIN_SENDING_TX; 
            }				
            break;

            case LIN_SENDING_TX:
			{ 
				if (s.write(LIN_TX_ID) == 1) {
	    			std::vector<uint8_t> v;
                    for (uint8_t i = 0; i < LIN_DATA_BYTES_NUM - 1; i++)                      // make-up your data to send
                    {
                        linMasterData[i] = 0x70 + i;
					    v.push_back(linMasterData[i])
                    }
                    linMasterData[LIN_DATA_BYTES_NUM - 1] = LIN_CalcCheckSum(linMasterData, LIN_DATA_BYTES_NUM - 1);
		    	    v.push_back(linMasterData[LIN_DATA_BYTES_NUM - 1])
                    if (s.write(v) == LIN_DATA_BYTES_NUM) {
			    		masterState = LIN_SENDING_DATA;
	    			} else {
    				    std::cout << "failed to send to the master" << std::endl;
                        masterState = LIN_SENDING_BREAK;
                    }
				}
            }
            break;
			
            case LIN_SENDING_DATA:
			{
                linMasterTxCnt++;
				std::cout << "written some bytes back to the master total msgs= " << linMasterTxCnt << std::endl;
                masterState = LIN_CLOSE_PORT;
			}
            break;

            case LIN_CLOSE_PORT:
			{
				if (s.write(LIN_SYNC_CLSE) == 1) {
			        std::cout << "close port stop was sent to slave communication closed" << std::endl;
   			        s.close();
				}
			}
            break;
				
            default:
			{
			    masterState = LIN_SENDING_BREAK;
			}
            break;
        }
}

int main(int argc, char **argv)
{
    // define the port charicturistics and open the device
    uint16_t baudRate = 115200;
    flowControlType flowControl = flowControlType::none;
    uint16_t characterSize = 8;
    parityType parity = parityType::none;
    stopBitsType stopBits = stopBitsType::one;

    s.open("/dev/ttyUSB1", baudRate, flowControl, characterSize, parity, stopBits);
    if (!s.isOpen())
        return 1;
    // if you want change baud rate then... s.setBaud(115200);
    s.flush();
    s.setTimeout(DEFAULT_TIMEOUT);

    // communicate in loop with the master	
    while (masterState != LIN_CLOSE_PORT) {
        LIN_Master();
	}

    return 0;
}