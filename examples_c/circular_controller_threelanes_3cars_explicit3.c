#include <stdio.h>
enum modes {Straight,Turn_left,Turn_right,Stop};    // tmp
enum lanes {Lane0, Lane1, Lane2};

struct State {
    double tau1;
    double yf1;
    double thetaf1;
    double v1; 
    int lane1;

    double tau2;
    double yf2;
    double thetaf2;
    double v2;
    int lane2;

    double tau3;
    double yf3;
    double thetaf3;
    double v3; 

    enum lanes vehicle_lane;
    enum modes vehicle_mode;
};

typedef struct State State;
    
State controller(State s) {
    double tau1 = s.tau1;
    double yf1 = s.yf1;
    double thetaf1 = s.thetaf1;
    double v1 = s.v1;
    int lane1 = s.lane1;
    double tau2 = s.tau2;
    double yf2 = s.yf2;
    double thetaf2 = s.thetaf2;
    double v2 = s.v2;
    int lane2 = s.lane2;

    double tau3 = s.tau3;
    double yf3 = s.yf3;
    double thetaf3 = s.thetaf3;
    double v3 = s.v3;
    enum lanes vehicle_lane = s.vehicle_lane;
    enum modes vehicle_mode = s.vehicle_mode;

    if (vehicle_lane == Lane0){
        if (vehicle_mode==Straight){
            if(
                (30*tau1-30*tau3<20 && 
                30*tau1-30*tau3>10 &&
                lane1 == 0 &&
                (30*tau2-30*tau3>40 || lane2!=1 || tau3-tau2>10/30)) ||
                (30*tau2-30*tau3<20 && 
                30*tau2-30*tau3>10 &&
                lane2 == 0 &&
                (30*tau1-30*tau3>40 || lane1!=1 || tau3-tau1>10/30))
            ){
                vehicle_mode = Turn_right;
            }
            if(
                (30*tau1-30*tau3<5 && 
                30*tau1-30*tau3>0 &&
                lane1 == 0) ||
                (30*tau2-30*tau3<5 && 
                30*tau2-30*tau3>0 &&
                lane2 == 0)
            ){
                vehicle_mode = Stop;
                v3 = 0;
            }
        }
        if (vehicle_mode==Turn_right){
            if(yf3<=-2.5){
                vehicle_lane = Lane1;
                vehicle_mode = Straight; 
                yf3 = yf3+3;
            }
        }
        if (vehicle_mode==Stop){
            if(
                (30*tau1-30*tau3>40 && lane1==0) ||
                (30*tau2-30*tau3>40 && lane2==0) ||
                (lane1!=0 && lane2!=0)
            ){
                vehicle_mode = Straight;
                v3 = 4;
            }
        }
    }
    if (vehicle_lane == Lane1){
        if (vehicle_mode==Straight) {
            if(
                (30*tau1-30*tau3<21 && 
                30*tau1-30*tau3>10 &&
                lane1 == 1 &&
                (30*tau2-30*tau3>40 || lane2!=0 || tau3-tau2>10/30)) ||
                (30*tau2-30*tau3<21 && 
                30*tau2-30*tau3>10 &&
                lane2 == 1 &&
                (30*tau1-30*tau3>40 || lane1!=0 || tau3-tau1>10/30))
            ){
                vehicle_mode = Turn_left;
            }
            if(
                (30*tau1-30*tau3<20 && 
                30*tau1-30*tau3>10 &&
                lane1 == 1 &&
                (30*tau2-30*tau3>40 || lane2!=2 || tau3-tau2>10/30)) ||
                (30*tau2-30*tau3<20 && 
                30*tau2-30*tau3>10 &&
                lane2 == 1 &&
                (30*tau1-30*tau3>40 || lane1!=2 || tau3-tau1>10/30))    
            ){
                vehicle_mode = Turn_right;
            }
            if(
                (30*tau1-30*tau3<5 && 
                30*tau1-30*tau3>0 &&
                lane1 == 1) ||
                (30*tau2-30*tau3<5 && 
                30*tau2-30*tau3>0 &&
                lane2 == 1)
            ){
                vehicle_mode = Stop;
                v3 = 0;
            }
        }
        if (vehicle_mode==Turn_left){
            if(yf3>=2.5){
                vehicle_lane = Lane0;
                vehicle_mode = Straight; 
                yf3 = yf3-3;
            }
        }
        if (vehicle_mode==Turn_right){
            if(yf3<=-2.5){
                vehicle_lane = Lane2;
                vehicle_mode = Straight;
                yf3 = yf3+3;
            }
        }
        if (vehicle_mode==Stop){
            if(
                (30*tau1-30*tau3>40 && lane1==1) ||
                (30*tau2-30*tau3>40 && lane2==1) ||
                (lane1!=1 && lane2!=1)
            ){
                vehicle_mode = Straight;
                v3 = 4;
            }
        }
    }   
    if (vehicle_lane == Lane2){
        if (vehicle_mode==Straight) {
            if(
                (30*tau1-30*tau3<20 && 
                30*tau1-30*tau3>10 &&
                lane1 == 2 &&
                (30*tau2-30*tau3>40 || lane2!=1 || tau3-tau2>10/30)) ||
                (30*tau2-30*tau3<20 && 
                30*tau2-30*tau3>10 &&
                lane2 == 2 &&
                (30*tau1-30*tau3>40 || lane1!=1 || tau3-tau1>10/30))    
            ){
                vehicle_mode = Turn_left;
            }                
            if(
                (30*tau1-30*tau3<5 && 
                30*tau1-30*tau3>0 &&
                lane1 == 2) ||
                (30*tau2-30*tau3<5 && 
                30*tau2-30*tau3>0 &&
                lane2 == 2)
            ){
                vehicle_mode = Stop;
                v3 = 0;
            }
        }
        if (vehicle_mode==Turn_left){
            if(yf3>=2.5){
                vehicle_lane = Lane1;
                yf3 = yf3-3;
                vehicle_mode = Straight;
            }
        }
        if (vehicle_mode==Stop){
            if(
                (30*tau1-30*tau3>40 && lane1==2) ||
                (30*tau2-30*tau3>40 && lane2==2) ||
                (lane1!=2 && lane2!=2)
            ){
                v3 = 4;
                vehicle_mode = Straight;
            }
        }
    } 
    s.vehicle_mode = vehicle_mode;
    s.vehicle_lane = vehicle_lane;
    s.yf3 = yf3;
    s.v3 = v3;
    return s;
}

int main(){
    State s;
    s = controller(s);
    printf("Hello, World!\n");
    return 0;
}