//
// ------------------ LIN protocol slave ------------------------------
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

uint16_t linSlaveTxCnt = 0u;
uint16_t linSlaveRxCnt = 0u;

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
    LIN_RECEIVING_BREAK     = 0x01,
    LIN_RECEIVING_SYNC      = 0x02,
    LIN_RECEIVING_ID        = 0x03,
    LIN_RECEIVING_DATA      = 0x04,
    LIN_SENDING_DATA        = 0x05,
    LIN_CLOSE_PORT          = 0x06,
} LIN_State;

// initialise the state engine
LIN_State slaveState = LIN_RECEIVING_BREAK;

// initialise the receive byte
uint8_t rxByte = 0;

// perfroms the LIN Slave actions
//
void LIN_Slave()
{
        uint8_t checkSum = 0;
        switch(slaveState)
        {
            case LIN_RECEIVING_BREAK:
            {
                slaveState = LIN_RECEIVING_SYNC;
				rxByte = s.read(1);
            }
            break;
			
            case LIN_RECEIVING_SYNC:
			{
                if (rxByte == LIN_SYNC_BYTE)
                {
                    slaveState = LIN_RECEIVING_ID;
                    rxByte = s.read(1);
                }
                else if (rxByte == LIN_SYNC_CLSE)
                {
                    slaveState = LIN_CLOSE_PORT;
                }
                else
                {
                    slaveState = LIN_RECEIVING_BREAK;
                }
            }
            break;

            case LIN_RECEIVING_ID:
			{
                if (rxByte == LIN_RX_ID)
                {
	    			std::vector<uint8_t> v;
                    for (uint8_t i = 0; i < LIN_DATA_BYTES_NUM - 1; i++)                      // make-up your data to send
                    {
                        linSlaveData[i] = 0x30 + i;
					    v.push_back(linSlaveData[i])
                    }
                    linSlaveData[LIN_DATA_BYTES_NUM - 1] = LIN_CalcCheckSum(linSlaveData, LIN_DATA_BYTES_NUM - 1);
		    	    v.push_back(linSlaveData[LIN_DATA_BYTES_NUM - 1])
                    if (s.write(v) == LIN_DATA_BYTES_NUM) {
			    		slaveState = LIN_SENDING_DATA;
	    			} else {
    				    std::cout << "failed to send to the master" << std::endl;
                        slaveState = LIN_RECEIVING_BREAK;
                    }					
                }
                else
                {
                    if (rxByte == LIN_TX_ID)
                    {
                        slaveState = LIN_RECEIVING_DATA;
                        linSlaveData, LIN_DATA_BYTES_NUM);
	    				std::vector<uint8_t> v = s.readBytes(LIN_DATA_BYTES_NUM);            // read the message bytes from the port
	    				for (uint8_t i = 0; i < LIN_DATA_BYTES_NUM - 1; i++)
                        {
                            linSlaveData[i] = v[i];                                          // v.at(i)
                        }
                    }
                    else
                    {
                        slaveState = LIN_RECEIVING_BREAK;
                    }
                }
			}
            break;

            case LIN_RECEIVING_DATA:
			{
                checkSum = LIN_CalcCheckSum(linSlaveData, LIN_DATA_BYTES_NUM - 1);
        
                if (linSlaveData[LIN_DATA_BYTES_NUM - 1] == checkSum)
                {
                    linSlaveRxCnt++;
				    std::cout << "slave received packet total packets= " << linSlaveRxCnt << std::endl;
                } else {
				    std::cout << "slave received packet with invalid checksum" << std::endl;			
			    }
        
                slaveState = LIN_RECEIVING_BREAK; 
            }				
            break;

            case LIN_SENDING_DATA:
			{
                linSlaveTxCnt++;
				std::cout << "written some bytes back to the master total msgs= " << linSlaveTxCnt << std::endl;
                slaveState = LIN_RECEIVING_BREAK;
			}
            break;

            case LIN_CLOSE_PORT:
			{
			    std::cout << "close port stop was received" << std::endl;
   			    s.close();
			}
            break;
				
            default:
			{
			    slaveState = LIN_RECEIVING_BREAK;
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

    s.open("/dev/ttyUSB0", baudRate, flowControl, characterSize, parity, stopBits);
    if (!s.isOpen())
        return 1;
    // s.setBaud(115200);
    s.flush();
    s.setTimeout(DEFAULT_TIMEOUT);

    //wait for master
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // communicate in loop with the master	
    while (slaveState != LIN_CLOSE_PORT) {
        LIN_Slave();
	}

    return 0;
}