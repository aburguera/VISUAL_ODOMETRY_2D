% Name        : plot_odometry(odoData)
% Description : Basic plot of odometry data
% Input       : odoData - Data structure provided by compute_odometry.
function plot_odometry(odoData)
    X=zeros(3,1);
    Xh=X;
    for i=1:size(odoData,2)
        [X,~]=compose_references(X,odoData(i).X,[],[]);
        Xh=[Xh X];
    end;
    
    figure;
    plot(Xh(1,:),Xh(2,:),'k');
    hold on;
    for i=1:10:size(Xh,2)
        draw_vehicle(Xh(:,i),50);
        hold on;
    end;
    drawnow;
    axis equal;
return;