
#include <stdio.h>
#include "PID.h"

FILE *ttt;

int sss=0;

void PIDLoc_t_test(){
	
	PIDLoc_t pid;
	float tmp = sss;
	int i=20;
	
	PIDLoc_init(&pid,0.96,0.31,0.05);
	
	pid.PID_stable(&pid,200);
	
	while(i--){
		
		tmp -= pid.PID_pop(&pid,tmp);
		fprintf(ttt,"%.2f\n",tmp);
		
	}
	
	
}
void PIDIncre_t_test(){
	
	PIDIncre_t pid;
	float tmp = sss;
	int i=20;
	
	PIDIncre_init(&pid,0.96,0.31,0.05);
	
	pid.PID_stable(&pid,200);
	
	while(i--){
		
		tmp -= pid.PID_pop(&pid,tmp);
		fprintf(ttt,"%.2f\n",tmp);
		
	}
	
	
}
void PIDIncreDT_t_test(){
	
	PIDIncreDT_t pid;
	float tmp = sss;
	int i=20;
	
	PIDIncreDT_init(&pid,0.96,0.31,0.05,1,3);
	
	pid.PID_stable(&pid,200);
	
	while(i--){
		
		tmp -= pid.PID_pop(&pid,tmp);
		fprintf(ttt,"%.2f\n",tmp);
		
	}
	
	
}
void PIDIncreDTB_t_test(){
	
	PIDIncreDTB_t pid;
	float tmp = sss;
	int i=20;
	
	PIDIncreDTB_init(&pid,0.96,0.31,0.05,1,3);
	
	pid.PID_stable(&pid,200);
	
	while(i--){
		
		tmp -= pid.PID_pop(&pid,tmp);
		fprintf(ttt,"%.2f\n",tmp);
		
	}
	
	
}
int main(){
	
	ttt = fopen("tt.txt","w+");
	
	if(ttt == NULL){
		
		printf("null\n");
		
	}
	
	//PIDLoc_t_test();
	//fprintf(ttt,"done\n");
	//PIDIncre_t_test();
	//fprintf(ttt,"done\n");
	PIDIncreDT_t_test();
	fprintf(ttt,"done\n");
	PIDIncreDTB_t_test();
	
	fclose(ttt);
	
	printf("done\n");
	
	return 0;
}


