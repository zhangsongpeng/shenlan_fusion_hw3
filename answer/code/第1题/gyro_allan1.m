clear 
close all

dt = dlmread('/home/zsp/Desktop/imu/allan1/data_imu_gyr_t.txt');         
data_x = dlmread('/home/zsp/Desktop/imu/allan1/data_imu_gyr_x.txt'); 
data_y= dlmread('/home/zsp/Desktop/imu/allan1/data_imu_gyr_y.txt'); 
data_z = dlmread('/home/zsp/Desktop/imu/allan1/data_imu_gyr_z.txt'); 
data_draw=[data_x data_y data_z]*pi/180/3600 ;

data_sim_x= dlmread('/home/zsp/Desktop/imu/allan1/data_imu_sim_gyr_x.txt'); 
data_sim_y= dlmread('/home/zsp/Desktop/imu/allan1/data_imu_sim_gyr_y.txt'); 
data_sim_z= dlmread('/home/zsp/Desktop/imu/allan1/data_imu_sim_gyr_z.txt'); 
data_sim_draw=[data_sim_x data_sim_y data_sim_z]*pi/180/3600 ;


figure
loglog(dt, data_draw , 'o');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
ylabel('Sigma:rad/s');             
% legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , '-');
