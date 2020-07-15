#ifndef home_meteo_h
#define home_meteo_h

#include <DS3231.h>

typedef enum {none_command=0,set_time,start_command,test_command}mode_commands;
typedef enum {
  start_state=0,  // -только включился и сделал первую передачу
  connecting_state, // -получил ack и проверяет устойчивость связи (подбор мощности передатчика
  connected_state,  // -связь установлена. Перешел в поминутный режим обмена
  lost_connectState,// -связь временно потеряна. Попытка восстановления
  disconnected_state,// -режим работы без связи
  test_state       // -специальный режим тестирования
}transmit_states;
typedef enum {none_q=0,get_time,get_meteoOutDoor,get_meteoInDoor}q_commands;
typedef enum {data_null,data_meteoOutDoor,data_meteoInDoor,exactly_time}data_type;


typedef enum {start_pipe=0,disconnected_pipe,connected_pipe,test_pipe}pipe_statuses;
typedef enum {power_normal,power_lowing,power_highing}power_mode;
struct meteo{
	uint32_t measurement_time;
	int16_t T;
	uint16_t P,H,CO2;
};
struct meteo_data_struct{
	uint32_t round_tripDelay;
	int32_t client_time;
	struct meteo meteo_data;
	uint8_t query;
	uint8_t type_of_data;
	uint8_t state;
	uint8_t power;
	uint8_t vcc;
	uint8_t vc1;
	uint8_t vc2;
	uint8_t vc3;
} ;
struct server_ack{
	uint32_t server_time;
	int32_t time_interval; //задержка перед следующей передачей
	struct meteo meteo_data;
	mode_commands command;
	q_commands ack_query;
	uint8_t channel,data_rate,power;
	uint8_t ac1;
};
struct pipe_data{
	pipe_statuses pipe_status;
	uint32_t lastConnectedTime;
	long int exchange_counter;
	int longer_exchange_counter; 
	power_mode power_tune;
	server_ack ackData;
};
struct sensors_local{
double T,P,H;
uint16_t lux;
RTCDateTime moment;
} ;

class RF24EventStack
{
private:
	uint8_t stack[12];
	uint8_t firstNo=0;
	uint8_t lastNo=0;
public:
	void push(uint8_t pipeNo);
	uint8_t pop();
	bool isEmpty();
};

void unixtimeToString(uint32_t unixT,char* str);
/*
class DS3231app : public DS3231
{
	public:
	bool addSeconds(RTCDateTime* dt,long int sec);
};
*/
/*
class remote_data_set
{
private:

protected:
	char name[10] = {0};
public:

}
*/
#endif
