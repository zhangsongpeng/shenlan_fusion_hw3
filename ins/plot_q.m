delimiterIn = ' ';
headerlinesIn = 0;
pose0 = importdata('pose_gt_00.txt', delimiterIn, headerlinesIn);
pose1 = importdata('pose_gt_11.txt', delimiterIn, headerlinesIn);
pose0 = abs(pose0) * 180 / pi;
pose1 = abs(pose1) * 180 / pi;
figure(1)
plot(pose0(:,1),pose0(:,5),'r',pose1(:,1),pose1(:,5),'b','LineWidth',2);
xlabel('time (sec)','FontSize',12,'FontWeight','bold')
ylabel('angle (deg)','FontSize',12,'FontWeight','bold')
legend('角增量','等效旋转矢量','FontSize',15);
set(gca,'FontSize',12);


