#include "route_steps.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define RAD (0.0174532925199432957692)

static bool valid(double lat,double lon){
    return isfinite(lat)&&isfinite(lon)&&fabs(lat)<=90&&fabs(lon)<=180;
}

RouteStepType route_step_type_from_osrm(const char *t){
    if(!t)return ROUTE_STEP_OTHER;
    static const struct {const char *k;RouteStepType v;} map[]={
        {"depart",ROUTE_STEP_DEPART},{"turn",ROUTE_STEP_TURN},
        {"new name",ROUTE_STEP_NEW_NAME},{"continue",ROUTE_STEP_CONTINUE},
        {"merge",ROUTE_STEP_MERGE},{"on ramp",ROUTE_STEP_ON_RAMP},
        {"off ramp",ROUTE_STEP_OFF_RAMP},{"fork",ROUTE_STEP_FORK},
        {"end of road",ROUTE_STEP_END_OF_ROAD},
        {"roundabout",ROUTE_STEP_ROUNDABOUT},{"rotary",ROUTE_STEP_ROUNDABOUT},
        {"roundabout turn",ROUTE_STEP_ROUNDABOUT},
        {"exit roundabout",ROUTE_STEP_EXIT_ROUNDABOUT},
        {"exit rotary",ROUTE_STEP_EXIT_ROUNDABOUT},
        {"arrive",ROUTE_STEP_ARRIVE}};
    for(size_t i=0;i<sizeof(map)/sizeof(map[0]);i++)
        if(!strcmp(t,map[i].k))return map[i].v;
    return ROUTE_STEP_OTHER;
}

RouteStepModifier route_step_modifier_from_osrm(const char *m){
    if(!m)return ROUTE_MOD_NONE;
    static const struct {const char *k;RouteStepModifier v;} map[]={
        {"uturn",ROUTE_MOD_UTURN},{"sharp right",ROUTE_MOD_SHARP_RIGHT},
        {"right",ROUTE_MOD_RIGHT},{"slight right",ROUTE_MOD_SLIGHT_RIGHT},
        {"straight",ROUTE_MOD_STRAIGHT},{"slight left",ROUTE_MOD_SLIGHT_LEFT},
        {"left",ROUTE_MOD_LEFT},{"sharp left",ROUTE_MOD_SHARP_LEFT}};
    for(size_t i=0;i<sizeof(map)/sizeof(map[0]);i++)
        if(!strcmp(m,map[i].k))return map[i].v;
    return ROUTE_MOD_NONE;
}

void route_steps_clear(RouteSteps *s){if(s){s->count=0;s->attached=false;}}

bool route_steps_add(RouteSteps *s,double lat,double lon,RouteStepType type,
                     RouteStepModifier modifier,int exit,const char *name){
    if(!s||s->count>=ROUTE_STEPS_MAX||!valid(lat,lon))return false;
    RouteStep *st=&s->steps[s->count++];
    memset(st,0,sizeof(*st));
    st->lat=lat;st->lon=lon;st->type=type;st->modifier=modifier;
    st->exit=exit>0&&exit<100?exit:0;
    if(name)snprintf(st->name,sizeof(st->name),"%s",name);
    s->attached=false;
    return true;
}

bool route_steps_attach(RouteSteps *s,const RouteFollow *r){
    if(!s||!r||!r->path||!r->metres||r->count<2)return false;
    size_t from=0;double prev=-2.;
    for(size_t k=0;k<s->count;k++){
        RouteStep *st=&s->steps[k];
        double c=cos(st->lat*RAD),best=INFINITY,along=0;
        size_t best_seg=from;
        for(size_t i=from;i+1<r->count;i++){
            RouteFollowPoint a=r->path[i],b=r->path[i+1];
            double ax=remainder(a.lon-st->lon,360.)*111195.*c,ay=(a.lat-st->lat)*111195.;
            double bx=remainder(b.lon-st->lon,360.)*111195.*c,by=(b.lat-st->lat)*111195.;
            double vx=bx-ax,vy=by-ay,den=vx*vx+vy*vy;
            double t=den>1e-8?fmax(0.,fmin(1.,-(ax*vx+ay*vy)/den)):0.;
            double dx=ax+t*vx,dy=ay+t*vy,d2=dx*dx+dy*dy;
            double at=r->metres[i]+t*(r->metres[i+1]-r->metres[i]);
            /* Consecutive maneuvers never share a spot on the route. */
            if(at<=prev+1.)continue;
            /* Maneuver points are route vertices, so the first match within a
             * couple of metres is the right one even if the route loops. */
            if(d2<best){best=d2;best_seg=i;along=at;}
            if(best<4.)break;
        }
        if(!isfinite(best))along=prev>0?prev:0; /* nothing ahead: keep order */
        st->along_m=along;
        from=best_seg;prev=along;
    }
    s->attached=true;
    return true;
}

const RouteStep *route_steps_next(const RouteSteps *s,const RouteFollow *r,
                                  double *distance_m){
    if(!s||!r||!s->attached||!r->matched||r->geometry_m<=0)return NULL;
    /* RouteFollow reports progress scaled to the provider's distance; the
     * maneuvers are placed on raw geometry metres, so scale back. */
    double scale=r->route_m>0?r->geometry_m/r->route_m:1.;
    double here=r->progress_m*scale;
    for(size_t k=0;k<s->count;k++){
        const RouteStep *st=&s->steps[k];
        if(st->type==ROUTE_STEP_DEPART)continue;
        /* A maneuver stays "next" until we are 8 m past it, so GPS jitter
         * right at the corner does not skip the instruction early. */
        if(st->along_m+8.>here){
            if(distance_m)*distance_m=fmax(0.,st->along_m-here)/scale;
            return st;
        }
    }
    return NULL;
}

void route_format_distance(double m,RouteLang lang,char *buf,size_t size){
    if(!buf||!size)return;
    if(!isfinite(m)||m<0)m=0;
    if(m<100)snprintf(buf,size,"%d m",(int)(round(m/10.)*10));
    else if(m<950)snprintf(buf,size,"%d m",(int)(round(m/50.)*50));
    else if(m<9950){
        snprintf(buf,size,"%.1f km",m/1000.);
        if(lang==ROUTE_LANG_PT_BR){char *dot=strchr(buf,'.');if(dot)*dot=',';}
    } else snprintf(buf,size,"%d km",(int)round(m/1000.));
}

/* --- Portuguese (Brazil) ------------------------------------------------ */
static const char *pt_side(RouteStepModifier m){
    switch(m){
    case ROUTE_MOD_SHARP_RIGHT:case ROUTE_MOD_RIGHT:case ROUTE_MOD_SLIGHT_RIGHT:return "à direita";
    case ROUTE_MOD_SHARP_LEFT:case ROUTE_MOD_LEFT:case ROUTE_MOD_SLIGHT_LEFT:return "à esquerda";
    default:return NULL;
    }
}
static const char *pt_turn(RouteStepModifier m){
    switch(m){
    case ROUTE_MOD_UTURN:return "Faça o retorno";
    case ROUTE_MOD_SHARP_RIGHT:return "Vire acentuadamente à direita";
    case ROUTE_MOD_RIGHT:return "Vire à direita";
    case ROUTE_MOD_SLIGHT_RIGHT:return "Vire levemente à direita";
    case ROUTE_MOD_SHARP_LEFT:return "Vire acentuadamente à esquerda";
    case ROUTE_MOD_LEFT:return "Vire à esquerda";
    case ROUTE_MOD_SLIGHT_LEFT:return "Vire levemente à esquerda";
    default:return "Siga em frente";
    }
}
/* "na Rua X", "no Viaduto Y", "em SP-280". */
static const char *pt_prep(const char *name){
    static const char *fem[]={"Rua","Avenida","Av.","Av ","Estrada","Rodovia","Alameda",
        "Travessa","Praça","Marginal","Ponte","Via ","Ladeira","Passarela"};
    static const char *masc[]={"Viaduto","Largo","Túnel","Acesso","Anel","Contorno",
        "Elevado","Corredor","Retorno"};
    for(size_t i=0;i<sizeof(masc)/sizeof(masc[0]);i++)
        if(!strncmp(name,masc[i],strlen(masc[i])))return "no";
    for(size_t i=0;i<sizeof(fem)/sizeof(fem[0]);i++)
        if(!strncmp(name,fem[i],strlen(fem[i])))return "na";
    return "em";
}
static void pt_action(const RouteStep *st,char *out,size_t n){
    const char *side=pt_side(st->modifier);
    switch(st->type){
    case ROUTE_STEP_TURN:snprintf(out,n,"%s",pt_turn(st->modifier));break;
    case ROUTE_STEP_END_OF_ROAD:
        snprintf(out,n,"No fim da via, %s",side?(st->modifier==ROUTE_MOD_LEFT||
            st->modifier==ROUTE_MOD_SLIGHT_LEFT||st->modifier==ROUTE_MOD_SHARP_LEFT?
            "vire à esquerda":"vire à direita"):"siga em frente");break;
    case ROUTE_STEP_FORK:
        if(side)snprintf(out,n,"Mantenha-se %s",side);
        else snprintf(out,n,"Siga em frente na bifurcação");
        break;
    case ROUTE_STEP_MERGE:
        if(side)snprintf(out,n,"Entre na via %s",side);else snprintf(out,n,"Entre na via");
        break;
    case ROUTE_STEP_ON_RAMP:
        if(side)snprintf(out,n,"Pegue o acesso %s",side);else snprintf(out,n,"Pegue o acesso");
        break;
    case ROUTE_STEP_OFF_RAMP:
        if(side)snprintf(out,n,"Pegue a saída %s",side);else snprintf(out,n,"Pegue a saída");
        break;
    case ROUTE_STEP_ROUNDABOUT:
        if(st->exit)snprintf(out,n,"Na rotatória, pegue a %dª saída",st->exit);
        else snprintf(out,n,"Entre na rotatória");
        break;
    case ROUTE_STEP_EXIT_ROUNDABOUT:snprintf(out,n,"Saia da rotatória");break;
    case ROUTE_STEP_ARRIVE:snprintf(out,n,"Chegada ao destino");break;
    case ROUTE_STEP_DEPART:snprintf(out,n,"Siga pela rota");break;
    case ROUTE_STEP_NEW_NAME:case ROUTE_STEP_CONTINUE:
        if(st->modifier==ROUTE_MOD_UTURN)snprintf(out,n,"Faça o retorno");
        else if(side)snprintf(out,n,"Continue %s",side);
        else snprintf(out,n,"Continue");
        break;
    default:
        if(st->modifier!=ROUTE_MOD_NONE)snprintf(out,n,"%s",pt_turn(st->modifier));
        else snprintf(out,n,"Continue");
        break;
    }
}

/* --- English -------------------------------------------------------------- */
static const char *en_dir(RouteStepModifier m){
    switch(m){
    case ROUTE_MOD_UTURN:return "make a U-turn";
    case ROUTE_MOD_SHARP_RIGHT:return "sharp right";
    case ROUTE_MOD_RIGHT:return "right";
    case ROUTE_MOD_SLIGHT_RIGHT:return "slight right";
    case ROUTE_MOD_SHARP_LEFT:return "sharp left";
    case ROUTE_MOD_LEFT:return "left";
    case ROUTE_MOD_SLIGHT_LEFT:return "slight left";
    case ROUTE_MOD_STRAIGHT:return "straight";
    default:return NULL;
    }
}
static void en_action(const RouteStep *st,char *out,size_t n){
    const char *d=en_dir(st->modifier);
    bool side=d&&st->modifier!=ROUTE_MOD_STRAIGHT&&st->modifier!=ROUTE_MOD_UTURN;
    switch(st->type){
    case ROUTE_STEP_TURN:
        if(st->modifier==ROUTE_MOD_UTURN)snprintf(out,n,"Make a U-turn");
        else if(side)snprintf(out,n,"Turn %s",d);else snprintf(out,n,"Go straight");
        break;
    case ROUTE_STEP_END_OF_ROAD:
        snprintf(out,n,"At the end of the road, %s%s",side?"turn ":"go ",side?d:"straight");break;
    case ROUTE_STEP_FORK:
        if(side)snprintf(out,n,"Keep %s at the fork",d);else snprintf(out,n,"Go straight at the fork");
        break;
    case ROUTE_STEP_MERGE:
        if(side)snprintf(out,n,"Merge %s",d);else snprintf(out,n,"Merge");break;
    case ROUTE_STEP_ON_RAMP:
        if(side)snprintf(out,n,"Take the ramp on the %s",d);else snprintf(out,n,"Take the ramp");break;
    case ROUTE_STEP_OFF_RAMP:
        if(side)snprintf(out,n,"Take the exit on the %s",d);else snprintf(out,n,"Take the exit");break;
    case ROUTE_STEP_ROUNDABOUT:{
        static const char *suf[]={"th","st","nd","rd"};
        int e=st->exit;const char *sx=(e%100>=11&&e%100<=13)||e%10>3?suf[0]:suf[e%10];
        if(e)snprintf(out,n,"At the roundabout, take the %d%s exit",e,sx);
        else snprintf(out,n,"Enter the roundabout");
        break;}
    case ROUTE_STEP_EXIT_ROUNDABOUT:snprintf(out,n,"Exit the roundabout");break;
    case ROUTE_STEP_ARRIVE:snprintf(out,n,"Arrive at destination");break;
    case ROUTE_STEP_DEPART:snprintf(out,n,"Head along the route");break;
    default:
        if(st->modifier==ROUTE_MOD_UTURN)snprintf(out,n,"Make a U-turn");
        else if(side)snprintf(out,n,"Continue %s",d);else snprintf(out,n,"Continue");
        break;
    }
}

bool route_step_format(const RouteStep *st,double distance_m,RouteLang lang,
                       char *buf,size_t size){
    if(!st||!buf||size<2)return false;
    char action[96],dist[24];
    if(lang==ROUTE_LANG_PT_BR)pt_action(st,action,sizeof(action));
    else en_action(st,action,sizeof(action));
    bool named=st->name[0]&&st->type!=ROUTE_STEP_ARRIVE&&
               st->type!=ROUTE_STEP_EXIT_ROUNDABOUT;
    int w;
    if(distance_m>=0){
        route_format_distance(distance_m,lang,dist,sizeof(dist));
        /* Lower-case the first ASCII letter after the distance prefix. */
        if(isupper((unsigned char)action[0]))action[0]=(char)tolower((unsigned char)action[0]);
        if(lang==ROUTE_LANG_PT_BR)w=named?snprintf(buf,size,"Em %s, %s %s %s",dist,action,pt_prep(st->name),st->name)
                                         :snprintf(buf,size,"Em %s, %s",dist,action);
        else w=named?snprintf(buf,size,"In %s, %s onto %s",dist,action,st->name)
                    :snprintf(buf,size,"In %s, %s",dist,action);
    } else {
        if(lang==ROUTE_LANG_PT_BR)w=named?snprintf(buf,size,"%s %s %s",action,pt_prep(st->name),st->name)
                                         :snprintf(buf,size,"%s",action);
        else w=named?snprintf(buf,size,"%s onto %s",action,st->name):snprintf(buf,size,"%s",action);
    }
    return w>=0&&(size_t)w<size;
}
