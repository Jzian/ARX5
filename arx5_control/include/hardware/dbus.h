/**
* 遥控器的数据
* DBus_Use()->d_bus_data.ch_r_y;  //右侧摇杆前后
* DBus_Use()->d_bus_data.ch_r_x;  //右侧摇杆左右
* DBus_Use()->d_bus_data.ch_l_y;  //左侧摇杆前后
* DBus_Use()->d_bus_data.ch_l_x;  //左侧摇杆左右
* DBus_Use()->d_bus_data.s_l;     //左拨钮
* DBus_Use()->d_bus_data.s_r;     //右拨钮
*/
#ifndef SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_
#define SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_

#include <cstdint>

typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s0;
    uint8_t s1;
    int16_t wheel;

    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
    uint16_t key;

} DBusData_t;

typedef struct
{

    uint8_t UP = 1;
    uint8_t DOWN = 2;
    uint8_t MID = 3;

    float ch_l_x; //左右-660  660
    float ch_l_y; //前后
    float ch_r_x; //左右-660  660
    float ch_r_y; //前后

    uint8_t s_l;
    uint8_t s_r;
    float wheel;
    //#mouse
    float m_x;
    float m_y;
    float m_z;
    bool p_l;
    bool p_r;
    //#key board
    bool key_w;
    bool key_s;
    bool key_a;
    bool key_d;
    bool key_shift;
    bool key_ctrl;
    bool key_q;
    bool key_e;
    bool key_r;
    bool key_f;
    bool key_g;
    bool key_z;
    bool key_x;
    bool key_c;
    bool key_v;
    bool key_b;

    // time stamp;

} DBusData_Use_t;

class DBus
{
public:
    DBus() = default;
    ~DBus() = default;
    // void init(const char* serial);
    void init();
    void getData();
    void read();

    DBusData_t d_bus_data_{};
    DBusData_Use_t d_bus_data{};
    int port_{};
    int16_t buff_[18]{};
    bool is_success{};
    bool is_update_ = false;
    void unpack();

    // std::string serial_port_;
};

DBus *DBus_Use();
void Release_DB_Hardware();

#endif // SRC_RM_BRIDGE_INCLUDE_RT_RT_DBUS_H_
