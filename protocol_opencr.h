#define MAX_INBUFFER_SIZE 23
#define MAX_OUTBUFFER_SIZE 11

const uint8_t BYTE_FIRST = 0;
const uint8_t BYTE_SIZE = 1;
const uint8_t BYTE_MESSAGE = 2;
const uint8_t BYTE_CHK = 3;
const uint8_t BYTE_CHK1 = 3;
const uint8_t BYTE_CHK2 = 4;
static constexpr uint16_t IN_BUFFER_SIZE = 43;
static constexpr uint16_t OUT_BUFFER_SIZE = 20;


class BaseTransmitter {
  private:
    typedef void(*CallbackV)(const BaseTransmitter&);
    uint8_t tx_buffer[OUT_BUFFER_SIZE];
    typedef float _left_velocity_type;
    typedef float _right_velocity_type;
    typedef uint8_t _count_sync_type;

    Stream & serial;

  public:
    _left_velocity_type left_velocity;
    _right_velocity_type right_velocity;
    _count_sync_type count_sync;


    BaseTransmitter(Stream & serial) : serial(serial), left_velocity(0), right_velocity(0), count_sync(0) {}

    int serialize(unsigned char * outbuffer) const {
      union {
        struct {
          float left_velocity;
          float right_velocity;
        } real;
        uint8_t base[8];
      } u_data;
      u_data.real.left_velocity = this->left_velocity;
      u_data.real.right_velocity = this->right_velocity;
      for (int i = 0; i < 8; i++)
        sprintf((char *)(outbuffer + 2 * i), "%c%c", ((u_data.base[i] >> 4) & 0x0F) + 48, (u_data.base[i] & 0x0F) + 48);
      return 16;
    }

    void flush() {
      int l = this->serialize(tx_buffer + 1);
      tx_buffer[0] = '#';
      int chk = 0;
      l++;
      for (int i = 1; i < l; i++)
        chk += tx_buffer[i];
      tx_buffer[l++] = (((255 - (chk % 256)) & 0xF0) >> 4) + 48;
      tx_buffer[l++] = ((255 - (chk % 256)) & 0x0F) + 48;


      serial.write(tx_buffer, l);
      //serial.flush();
    }



};


class BaseReceiver {
  private:
    typedef void(*CallbackV)(const BaseReceiver&);
    Stream & serial;
    uint8_t rx_buffer[IN_BUFFER_SIZE];
    bool tx_cplt = true;

    CallbackV callbackV;
    int mode = BYTE_FIRST;
    int chk;
    int index_ = 0;
    int size_msg;

  public:
    typedef float _vel_type;
    typedef float _pos_type;
    typedef float _yaw_type;
    typedef uint16_t _bat_vel_type;

    _bat_vel_type bat_vel;

    _vel_type left_vel;
    _vel_type right_vel;
    _pos_type left_pos;
    _pos_type right_pos;
    _yaw_type yaw;

    uint64_t receive_success_msgs;
    uint64_t receive_fail_msgs;

    BaseReceiver(Stream & serial, CallbackV callbackV) : receive_success_msgs(0), receive_fail_msgs(0), serial(serial), callbackV(callbackV), bat_vel(0), left_vel(0), right_vel(0), left_pos(0), right_pos(0), yaw(0) {}

    int deserialize(unsigned char * inbuffer) {
      union {
        struct {
          float left_vel;
          float right_vel;
          float left_pos;
          float right_pos;
          float yaw;
          uint16_t bat_vel;
        } real;
        uint8_t base[22];
      } u_data;

      for (int i = 0; i < 22; i++)
        *(u_data.base + i) = ((*(inbuffer + 2 * i) - 48) << 4) | (*(inbuffer + 2 * i + 1) - 48);

      this->left_vel = u_data.real.left_vel;
      this->right_vel = u_data.real.right_vel;
      this->left_pos = u_data.real.left_pos;
      this->right_pos = u_data.real.right_pos;
      this->yaw = u_data.real.yaw;
      this->bat_vel = u_data.real.bat_vel;
      return 44;
    }
    void spin() {
      while (true) {
        int data = serial.read();

        if (data < 0) {
          break;
        }

        if (mode == BYTE_MESSAGE) {
          chk += data;
          rx_buffer[index_++] = data;
          size_msg--;
          if (size_msg == 0)
            mode = BYTE_CHK1;
        } else if (mode == BYTE_FIRST) {
          if (data == '#') {
            size_msg = 44;
            index_ = 0;
            mode = BYTE_MESSAGE;
          }
        } else if (mode == BYTE_CHK1) {
          chk += ((data - 48) & 0x0F) << 4;
          mode = BYTE_CHK2;
        } else if (mode == BYTE_CHK2) {
          chk += (data - 48) & 0x0F;
          mode = BYTE_FIRST;

          if ((chk % 256) == 255) {
            this->deserialize(rx_buffer);
            receive_success_msgs++;
            callbackV(*this);
          } else {
            receive_fail_msgs++;
          }
          chk = 0;
        }

      }
    }
};
