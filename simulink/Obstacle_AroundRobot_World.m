%DataTodo = simout';
%[~,number] = size(DataTodo);
%list = [1:1:number];
%plot(list,DataTodo);

%�ӻ����˺������۾��￴��Χ����

DataTodo = simout;
[~,~,Number] = size(DataTodo);
for i = 1:1:Number
    ToDoData = DataTodo(:,:,i);
    ToDoData = ToDoData(1:2,:);
    plot(ToDoData(1,:),ToDoData(2,:));
    hold on;
    plot(0,0,'r');
    hold off;
    pause(0.1);
end
%plot(ToDoData(1,:),ToDoData(2,:));
Temp = 0;