#include "route_steps.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static RouteSteps s;
int main(void){
 char b[160];
 assert(route_step_type_from_osrm("turn")==ROUTE_STEP_TURN);
 assert(route_step_type_from_osrm("rotary")==ROUTE_STEP_ROUNDABOUT);
 assert(route_step_type_from_osrm("weird")==ROUTE_STEP_OTHER);
 assert(route_step_type_from_osrm(NULL)==ROUTE_STEP_OTHER);
 assert(route_step_modifier_from_osrm("slight left")==ROUTE_MOD_SLIGHT_LEFT);
 assert(route_step_modifier_from_osrm(NULL)==ROUTE_MOD_NONE);

 route_format_distance(43,ROUTE_LANG_PT_BR,b,sizeof b);assert(!strcmp(b,"40 m"));
 route_format_distance(212,ROUTE_LANG_PT_BR,b,sizeof b);assert(!strcmp(b,"200 m"));
 route_format_distance(1240,ROUTE_LANG_PT_BR,b,sizeof b);assert(!strcmp(b,"1,2 km"));
 route_format_distance(1240,ROUTE_LANG_EN,b,sizeof b);assert(!strcmp(b,"1.2 km"));
 route_format_distance(23400,ROUTE_LANG_EN,b,sizeof b);assert(!strcmp(b,"23 km"));
 route_format_distance(NAN,ROUTE_LANG_EN,b,sizeof b);assert(!strcmp(b,"0 m"));

 /* L-shaped route: east ~853 m, then north ~1112 m. */
 RouteFollowPoint p[]={{40.0,-74.0},{40.0,-73.99},{40.01,-73.99}};
 RouteFollow r={0};
 assert(route_follow_begin(&r,p,3,0,600));
 route_steps_clear(&s);
 assert(route_steps_add(&s,40.0,-74.0,ROUTE_STEP_DEPART,ROUTE_MOD_NONE,0,"Rua A"));
 assert(route_steps_add(&s,40.0,-73.99,ROUTE_STEP_TURN,ROUTE_MOD_LEFT,0,"Avenida Paulista"));
 assert(route_steps_add(&s,40.01,-73.99,ROUTE_STEP_ARRIVE,ROUTE_MOD_NONE,0,"Avenida Paulista"));
 assert(!route_steps_add(&s,NAN,0,ROUTE_STEP_TURN,ROUTE_MOD_LEFT,0,NULL));
 assert(route_steps_next(&s,&r,NULL)==NULL); /* not attached / not matched */
 assert(route_steps_attach(&s,&r));
 assert(fabs(s.steps[1].along_m-r.metres[1])<1);
 double d=-1;
 assert(route_follow_update(&r,40.0,-74.0,5,1000));
 const RouteStep *n=route_steps_next(&s,&r,&d);
 assert(n==&s.steps[1] && fabs(d-r.metres[1])<2);
 assert(route_step_format(n,d,ROUTE_LANG_PT_BR,b,sizeof b));
 assert(!strcmp(b,"Em 850 m, vire à esquerda na Avenida Paulista"));
 assert(route_step_format(n,d,ROUTE_LANG_EN,b,sizeof b));
 assert(!strcmp(b,"In 850 m, turn left onto Avenida Paulista"));
 assert(route_follow_update(&r,40.0,-73.9905,5,2000));
 n=route_steps_next(&s,&r,&d);assert(n==&s.steps[1] && d<60 && d>30);
 /* Just past the corner (jitter) still shows the turn, further on the arrival. */
 assert(route_follow_update(&r,40.00003,-73.99,5,3000));
 n=route_steps_next(&s,&r,&d);assert(n==&s.steps[1] && d==0);
 assert(route_follow_update(&r,40.002,-73.99,5,4000));
 n=route_steps_next(&s,&r,&d);assert(n==&s.steps[2]);
 assert(route_step_format(n,d,ROUTE_LANG_PT_BR,b,sizeof b));
 assert(!strcmp(b,"Em 900 m, chegada ao destino"));
 assert(route_follow_update(&r,40.01,-73.99,5,5000));
 n=route_steps_next(&s,&r,&d);assert(n==&s.steps[2] && d<1);

 /* Provider distance differs from geometry: distances are reported in provider metres. */
 RouteFollow r2={0};assert(route_follow_begin(&r2,p,3,r.geometry_m*2,600));
 assert(route_steps_attach(&s,&r2));
 assert(route_follow_update(&r2,40.0,-74.0,5,1000));
 n=route_steps_next(&s,&r2,&d);assert(n==&s.steps[1] && fabs(d-2*r.metres[1])<4);
 route_follow_reset(&r2);

 /* Loop route passing the same corner twice keeps maneuvers in order. */
 RouteFollowPoint loop[]={{0,0},{0,0.01},{0.01,0.01},{0.01,0},{0,0},{0,0.01},{-0.01,0.01}};
 RouteFollow r3={0};assert(route_follow_begin(&r3,loop,7,0,0));
 route_steps_clear(&s);
 assert(route_steps_add(&s,0,0.01,ROUTE_STEP_TURN,ROUTE_MOD_LEFT,0,NULL));
 assert(route_steps_add(&s,0,0.01,ROUTE_STEP_TURN,ROUTE_MOD_RIGHT,0,NULL));
 assert(route_steps_attach(&s,&r3));
 assert(s.steps[1].along_m>s.steps[0].along_m+3000);
 route_follow_reset(&r3);

 RouteStep st={0};
 st.type=ROUTE_STEP_ROUNDABOUT;st.exit=2;snprintf(st.name,sizeof st.name,"Viaduto do Chá");
 assert(route_step_format(&st,-1,ROUTE_LANG_PT_BR,b,sizeof b));
 assert(!strcmp(b,"Na rotatória, pegue a 2ª saída no Viaduto do Chá"));
 assert(route_step_format(&st,-1,ROUTE_LANG_EN,b,sizeof b));
 assert(!strcmp(b,"At the roundabout, take the 2nd exit onto Viaduto do Chá"));
 st.exit=12;assert(route_step_format(&st,-1,ROUTE_LANG_EN,b,sizeof b));
 assert(strstr(b,"12th"));
 st.type=ROUTE_STEP_FORK;st.modifier=ROUTE_MOD_SLIGHT_RIGHT;snprintf(st.name,sizeof st.name,"SP-280");
 assert(route_step_format(&st,300,ROUTE_LANG_PT_BR,b,sizeof b));
 assert(!strcmp(b,"Em 300 m, mantenha-se à direita em SP-280"));
 st.type=ROUTE_STEP_END_OF_ROAD;st.modifier=ROUTE_MOD_LEFT;st.name[0]=0;
 assert(route_step_format(&st,-1,ROUTE_LANG_PT_BR,b,sizeof b));
 assert(!strcmp(b,"No fim da via, vire à esquerda"));
 st.type=ROUTE_STEP_TURN;st.modifier=ROUTE_MOD_UTURN;
 assert(route_step_format(&st,-1,ROUTE_LANG_PT_BR,b,sizeof b));assert(!strcmp(b,"Faça o retorno"));
 assert(!route_step_format(&st,-1,ROUTE_LANG_PT_BR,b,4));
 route_follow_reset(&r);
 puts("Turn-by-turn maneuver placement, next-instruction and pt-BR/EN text tests passed");
 return 0;
}
