#include <lwp.h>
#include <lwp_drivers.h>
#include <math.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>



float _map(float v,float aa,float ab,float ba,float bb){
	return (v-aa)/(ab-aa)*(bb-ba)+ba;
}



void _lwp_driver_base_motor_init(lwp_device_t* d,uint8_t p){
	lwp_setup_port(d,p,2,10,1);
	if ((d->ports.dt+p)->_drv_dt){
		free((d->ports.dt+p)->_drv_dt);
	}
	(d->ports.dt+p)->_drv_dt=malloc(sizeof(uint32_t));
}



void _lwp_driver_base_motor_update(lwp_device_t* d,uint8_t p,uint32_t f,float tm,void* bf){
	*((int32_t*)(d->ports.dt+p)->_drv_dt)=*((int32_t*)bf);
}



void lwp_driver_base_motor_goto_absolute_position(lwp_device_t* d,uint8_t p,int32_t ap,int8_t sp,uint8_t mp,uint8_t e_st,uint8_t pr){
	uint8_t bf[12]={LWP_COMMAND_PORT_OUTPUT_COMMAND,p,LWP_PORT_OUTPUT_FORMAT_NO_BUFFER|LWP_PORT_OUTPUT_FORMAT_NO_FEEDBACK,0x0d,ap&0xff,(ap>>8)&0xff,(ap>>16)&0xff,ap>>24,sp&255,mp,e_st,pr};
	lwp_send_raw_data(d,bf,sizeof(bf));
}



void _lwp_driver_led_light_init(lwp_device_t* d,uint8_t p){
	lwp_setup_port(d,p,1,0,0);
}



void lwp_driver_led_light_set_color(lwp_device_t* d,uint8_t p,uint8_t r,uint8_t g,uint8_t b){
	uint8_t bf[8]={LWP_COMMAND_PORT_OUTPUT_COMMAND,p,LWP_PORT_OUTPUT_FORMAT_NO_BUFFER|LWP_PORT_OUTPUT_FORMAT_NO_FEEDBACK,LWP_PORT_WRITE_DIRECT_MODE_DATA,0x01,r,g,b};
	lwp_send_raw_data(d,bf,sizeof(bf));
}



void _lwp_driver_gyro_init(lwp_device_t* d,uint8_t p){
	(d->ports.dt+p)->_drv_dt=NULL;
}



void lwp_driver_gyro_setup(lwp_device_t* d,uint8_t p,uint32_t ul){
	lwp_setup_port(d,p,0,ul,1);
	if ((d->ports.dt+p)->_drv_dt){
		free((d->ports.dt+p)->_drv_dt);
	}
	(d->ports.dt+p)->_drv_dt=NULL;
}



void _lwp_driver_gyro_update(lwp_device_t* d,uint8_t p,uint32_t f,float tm,void* bf){
	uint8_t dt_t=(uint8_t)(f>>16);
	uint8_t dt_l=(uint8_t)((f>>8)&0xff);
	uint8_t mi=(uint8_t)(f&0xff);
	int16_t* dt=(int16_t*)bf;
	float* p_dt=(d->ports.dt+p)->_drv_dt;
	if (!p_dt){
		(d->ports.dt+p)->_drv_dt=malloc(6*sizeof(float));
		p_dt=(d->ports.dt+p)->_drv_dt;
		*(p_dt)=0;
		*(p_dt+1)=0;
		*(p_dt+2)=0;
		*(p_dt+3)=*dt;
		*(p_dt+4)=*(dt+1);
		*(p_dt+5)=*(dt+2);
	}
	lwp_device_port_mode_range_t rr=(d->ports.dt+p)->ml[mi].rr;
	lwp_device_port_mode_range_t sr=(d->ports.dt+p)->ml[mi].sr;
	*p_dt+=roundf(_map((float)*dt-*(p_dt+3),rr.mn,rr.mx,sr.mn,sr.mx)*0.0127f);
	*(p_dt+1)+=roundf(_map((float)*(dt+1)-*(p_dt+4),rr.mn,rr.mx,sr.mn,sr.mx)*0.0127f);
	*(p_dt+2)+=roundf(_map((float)*(dt+2)-*(p_dt+5),rr.mn,rr.mx,sr.mn,sr.mx)*0.0127f);
}
