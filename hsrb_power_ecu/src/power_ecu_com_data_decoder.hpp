/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef POWER_ECU_COM_DATA_DECODER_HPP_
#define POWER_ECU_COM_DATA_DECODER_HPP_
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <boost/system/error_code.hpp>

#include "power_ecu_com_common.hpp"
#include "power_ecu_com_element_decoder.hpp"
#include "power_ecu_com_frame_decoder.hpp"

namespace hsrb_power_ecu {
/* *
* @brief Decoder for ecu1 command
*/
class PowerEcuComEcu1DataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComEcu1DataDecoder(PowerEcuComEcu1DataDecoder const&);             // = delete;
  PowerEcuComEcu1DataDecoder& operator=(PowerEcuComEcu1DataDecoder const&);  // = delete;

 public:
  /**
   * @brief Member variable pointer container for hw class of ecu1 command
   */
  struct PacketData {
    uint32_t time_stamp;                             //!< Timestamp [ms]
    std::string ecu1_date;                           //!< Date YYYYMMDDhhmmss
    std::string power_ecu_status_flag;               //!< Power ECU status part of S
    std::string diag_status;                         //!< Diagnostic information 32-digit hexadecimal
    double battery_total_capacity;                   //!< Battery total capacity [mAh]
    double battery_remaining_capacity;               //!< Battery remaining capacity [mAh]
    double electric_current;                         //!< Electric current value [mA]
    double battery_voltage;                          //!< Battery voltage [mV]
    double battery_temperature;                      //!< Battery temperature [C]
    bool is_battery_crgov;                           //!< Overcharge 1: Overcharge
    bool is_battery_23par;                           //!< Parallel number 0: 2 parallel 1: 3 parallel
    bool is_battery_std;                             //!< Learning permission 1: Learning allowed
    bool is_battery_full;                            //!< Full charge 1: Fully charged state
    bool is_battery_discov;                          //!< Overdischarge 1: Overdischarged
    bool is_battery_chg;                             //!< Charge permission 1: Charging allowed
    bool is_battery_disc;                            //!< Discharge permission 1: Discharge allowed
    bool is_battery_0per;                            //!< 0% Detection 1: 0% detected state
    bool is_battery_45par;                           //!< Parallel number 0: 4 parallel 1: 5 parallel
    bool is_battery_sel;                             //!< Minimum cell voltage 0% detection state 1: 0% detected state
    bool is_battery_bal;                             //!< Cell balance collapse 1: Unbalanced
    uint16_t battery_initial_learning_capacity;      //!< Battery initial learning capacity [mAh]
    uint16_t battery_error_status;                   //!< Battery error status specification undefined
    double battery_relative_capacity;                //!< Relative capacity [%]
    uint32_t power_ecu_internal_state;               //!< (New) Number of S** in 4.5.3 Power ECU internal state
    bool is_powerecu_bat_stat;                       //!< (New) Battery charging state
    bool is_powerecu_sw_kinoko;                      //!< (New) Wired emergency stop SW
    bool is_powerecu_sw_pwr;                         //!< (New) Power (Prius SW)
    bool is_powerecu_sw_drv;                         //!< (New) Drive SW
    bool is_powerecu_sw_latch;                       //!< (New) Latch release SW
    bool is_powerecu_sw_w_sel;                       //!< (New) Wireless switch SW
    bool is_powerecu_sw_w_stop;                      //!< (New) Wireless emergency stop SW
    bool is_bumper_bumper2;                          //!< Bumper sensor state 2 1: Contact detected
    bool is_bumper_bumper1;                          //!< Bumper sensor state 1 1: Contact detected
    bool is_bumper_prox5;                            //!< Proximity sensor latch state 5 1: Proximity detected
    bool is_bumper_prox4;                            //!< Proximity sensor latch state 4 1: Proximity detected
    bool is_bumper_prox3;                            //!< Proximity sensor latch state 3 1: Proximity detected
    bool is_bumper_prox2;                            //!< Proximity sensor latch state 2 1: Proximity detected
    bool is_bumper_prox1;                            //!< Proximity sensor latch state 1 1: Proximity detected
    std::string gyro_status;                         //!< (New) Gyro posture angle calculation status
    boost::array<double, 4> imu_quaternions;         //!< quaternion x,y,z,t -1.0~1.0
    boost::array<double, 3> imu_angular_velocities;  //!< Angular velocities x,y,z [rad/s]
    boost::array<double, 3> imu_accelerations;       //!< Accelerations x,y,z [m/s^2]
    uint8_t charger_state;                           //!< Automatic charging status
  };

 private:
  /**
   * @brief Internal container for receiving packet data
   */
  struct PacketRawData {
    uint32_t time_stamp;                         //!< Timestamp 10-digit decimal
    std::string date;                            //!< Date string
    std::string power_ecu_status_flag;           //!< Power ECU status 2-digit hexadecimal
    std::string power_ecu_status;                //!< (New) Power ECU status 16-digit hexadecimal
    std::string diag_status;                     //!< uint816 diagnostic information 32-digit hexadecimal
    uint16_t battery_total_capacity;             //!< Battery total capacity sign 5-digit decimal
    uint16_t battery_remaining_capacity;         //!< Battery remaining capacity sign 5-digit decimal
    int16_t electric_current;                    //!< Electric current value sign 5-digit decimal
    uint16_t battery_voltage;                    //!< Battery voltage sign 5-digit decimal
    int8_t battery_temperature;                  //!< Battery temperature sign 3-digit decimal
    uint16_t battery_state_flag;                 //!< Battery state flag 4-digit hexadecimal
    uint16_t battery_initial_learning_capacity;  //!< Battery initial learning capacity 5-digit decimal
    uint16_t battery_error_status;               //!< Battery error status 4-digit hexadecimal
    uint8_t battery_relative_capacity;           //!< Relative capacity 3-digit decimal
    uint8_t bumper_status;                       //!< Proximity, bumper sensor state 2-digit hexadecimal
    std::string gyro_status;                     //!< (New) Gyro posture angle calculation status 16-digit hexadecimal
    int32_t quaternion_t;                        //!< quaternion_t sign 10-digit decimal
    int32_t quaternion_x;                        //!< quaternion_x sign 10-digit decimal
    int32_t quaternion_y;                        //!< quaternion_y sign 10-digit decimal
    int32_t quaternion_z;                        //!< quaternion_z sign 10-digit decimal
    int32_t angular_velocity_x;                  //!< Angular velocity x sign 10-digit decimal
    int32_t angular_velocity_y;                  //!< Angular velocity y sign 10-digit decimal
    int32_t angular_velocity_z;                  //!< Angular velocity z sign 10-digit decimal
    int32_t acceleration_x;                      //!< Acceleration x sign 10-digit decimal
    int32_t acceleration_y;                      //!< Acceleration y sign 10-digit decimal
    int32_t acceleration_z;                      //!< Acceleration z sign 10-digit decimal
    uint8_t charger_state;                       //!< Automatic charging status
  };

 public:
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComEcu1DataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComEcu1DataDecoder() {}

 private:
  /**
   * @brief Post-decoding processing
   * @return True on success
   */
  virtual bool Update();

  PacketRawData packet_raw_data_;  //!< Hw class variable pointer container
  PacketData packet_out_;          //!< Buffer for receiving data
  std::string temp_str;            //!< Temporary
  uint32_t temp_uint;
};

/**
* @brief Decoder for ecu2 command
*/
class PowerEcuComEcu2DataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComEcu2DataDecoder(PowerEcuComEcu2DataDecoder const&);             // = delete;
  PowerEcuComEcu2DataDecoder& operator=(PowerEcuComEcu2DataDecoder const&);  // = delete;

 public:
  /**
  * @brief Packet data for ecu2 command
  */
  struct PacketData {
    std::string ecu2_date;               //!< Date (YYYYMMDDhhmmss) string
    uint16_t d12V_D0_V;                  //!< 12Vd0 voltage [mV] Sign 1 digit + 5-digit decimal
    int16_t d12V_D0_A;                   //!< 12Vd0 current [mA] Sign 1 digit + 5-digit decimal
    uint16_t d12V_D1_V;                  //!< 12Vd1 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d12V_D1_A;                   //!< 12Vd1 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d12V_D2_V;                  //!< 12Vd2 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d12V_D2_A;                   //!< 12Vd2 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d12V_D3_V;                  //!< 12Vd3 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d12V_D3_A;                   //!< 12Vd3 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d12V_O1_V;                  //!< 12Vo1 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d12V_O1_A;                   //!< 12Vo1 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d12V_O2_V;                  //!< 12Vo2 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d12V_O2_A;                   //!< 12Vo2 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d5VA_V;                     //!< 5Va voltage [mV] Sign 1 digit 5-digit decimal
    uint16_t d5VD1_V;                    //!< 5Vd1 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d5VD1_A;                     //!< 5Vd1 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d5VD2_V;                    //!< 5Vd2 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d5VD2_A;                     //!< 5Vd2 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d5VD3_V;                    //!< 5Vd3 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d5VD3_A;                     //!< 5Vd3 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d5VD4_V;                    //!< 5Vd4 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d5VD4_A;                     //!< 5Vd4 current [mA] Sign 1 digit 5-digit decimal
    uint16_t d5VD5_V;                    //!< 5Vd5 voltage [mV] Sign 1 digit 5-digit decimal
    int16_t d5VD5_A;                     //!< 5Vd5 current [mA] Sign 1 digit 5-digit decimal
    uint16_t Chgsense;                   //!< Automatic charging insert/remove terminal voltage [mV] Sign 1 digit 5-digit decimal
    uint16_t d2V5VDA1_V;                 //!< 2.5Va1 voltage (A/D1) [mV] Sign 1 digit 5-digit decimal
    uint16_t d2V5VDA2_V;                 //!< 2.5Va2 voltage (A/D2) [mV] Sign 1 digit 5-digit decimal
    uint16_t ACDC_V;                     //!< ACDC voltage [mV] Sign 1 digit 5-digit decimal
    int16_t ADCD_A;                      //!< ACDC current [mA] Sign 1 digit 5-digit decimal
    uint16_t BATT_V;                     //!< BATT voltage [mV] Sign 1 digit 5-digit decimal
    int16_t BATT_A;                      //!< BATT current [mA] Sign 1 digit 5-digit decimal
    int16_t BATT_A2;                     //!< BATT current 2 [10mA] Sign 1 digit 5-digit decimal
    uint16_t PBM_V;                      //!< PBM voltage [mV] Sign 1 digit 5-digit decimal
    int16_t PBM_A;                       //!< PBM current [mA] Sign 1 digit 5-digit decimal
    int16_t PBM_A2;                      //!< PBM current 2 [10mA] Sign 1 digit 5-digit decimal
    uint16_t PUMP_V;                     //!< Pump sensor voltage [mV] Sign 1 digit 5-digit decimal
    int16_t ECU_TEMP;                    //!< Power ECU temperature [°C] Sign 1 digit 3-digit decimal
    int16_t ECU_TEMP1;                   //!< Power ECU temperature 1 [°C] Sign 1 digit 3-digit decimal
    int16_t ECU_TEMP2;                   //!< Power ECU temperature 2 [°C] Sign 1 digit 3-digit decimal
    int16_t ECU_TEMP3;                   //!< Power ECU temperature 3 [°C] Sign 1 digit 3-digit decimal
  };

  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComEcu2DataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComEcu2DataDecoder() {}

 private:
  /**
   * @brief Post-decoding processing
   * @return True on success
   */
  virtual bool Update();
  PacketData packet_data_;  //!< Packet data
};

/**
 * @brief Decoder for RXACK command
 */
class PowerEcuComRxackDataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComRxackDataDecoder(PowerEcuComRxackDataDecoder const&);             // = delete;
  PowerEcuComRxackDataDecoder& operator=(PowerEcuComRxackDataDecoder const&);  // = delete;

 public:
  /**
   * @brief Packet data for Rxack
   */
  struct PacketData {
    /**
     * @brief Constructor
     */
    PacketData() : is_receive_ack(false), ack_value(0) {}
    bool is_receive_ack;  //!< Whether Ack has been returned
    uint8_t ack_value;    //!< Return value of the reply command
  };

  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComRxackDataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComRxackDataDecoder() {}

 private:
  /**
   * @brief Post-decoding processing
   * @return True on success
   */
  virtual bool Update();

  PacketData packet_data_;  //!< Packet data
};

/**
 * @brief Decoder for Ver command
 */
class PowerEcuComVerDataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComVerDataDecoder(PowerEcuComVerDataDecoder const&);             // = delete;
  PowerEcuComVerDataDecoder& operator=(PowerEcuComVerDataDecoder const&);  // = delete;

 public:
  /**
   * @brief Packet data for Ver
   */
  struct PacketData {
    PacketData() : is_receive_version(false) {}
    std::string ver_power_ecu_version;      //!< Power ECU firmware Ver [git hash 20byte] 40-digit hexadecimal
    std::string ver_power_ecu_com_version;  //!< Power ECU communication structure HASH [hash 20byte] 40-digit hexadecimal
    bool is_receive_version;                //!< Whether the Ver command has been received
  };

  /**
   * @brief Constructor
   * @param[in] packet_data Packet data
   */
  PowerEcuComVerDataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComVerDataDecoder() {}

 private:
  /**
   * @brief Post-decoding processing
   * @return True on success
   */
  virtual bool Update();

  PacketData packet_data_;                 //!< Packet data
  std::string power_ecu_version_raw_;      //!< Power ECU firmware Ver [git hash 20byte] 40-digit hexadecimal
  std::string power_ecu_com_version_raw_;  //!< Power ECU communication structure HASH [hash 20byte] 40-digit hexadecimal
};

}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_DATA_DECODER_HPP_
