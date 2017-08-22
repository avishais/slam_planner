#include "../include/camera.h"
#include <iostream>

int main(int argc, char *argv[]){
	if(argc!=4){

		std::cout<<argc<<"Input file addresses in order of 1. Map in SLAM frame, 2. upper_bound, 3. lower_bound"<<std::endl;
		return 0;
	}
	float x_w=-0.546;
	float y_w=0.2569;
	float theta_w=0.2094;
	clock_t t=clock();
	camera test(argv[1],argv[2],argv[3]);
	clock_t t1=clock();
	std::cout<<"Time for loading map is "<<((float)(t1-t)/CLOCKS_PER_SEC)<<" s"<<std::endl;
	//input : x in world frame, y in world frame, theta in world frame
	int num_obs=test.countVisible(x_w,y_w,theta_w);
	std::cout<<"Time for calculate one pose is "<<((float)(clock()-t1)/CLOCKS_PER_SEC)<<" s"<<std::endl;
	std::cout<<"X_w: "<<x_w<<", Y_w: "<<y_w<<", theta_w: "<<theta_w<<", # of visible points: "<<num_obs<<std::endl;

    
    return 0;
}