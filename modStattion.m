% системы из 3 элеменитов 
% clear; 
close all;

c = 1500; % скорость звука
fs = 48e3; % частота дискретизации
nwin = 2^10;% размер выборки(число точек Фурье)
dt = 1 / fs; % шаг по времени
d = 1.0; % расстояние между приемниками
n = 2; % кол-во АР;

distance = 1000; % расстояние между АР
M=sqrt(d^2-(d/2)^2); % медиана треугольника

% координаты 1 АР
% координаты приемных элементов АР
xAnt(1,:) = [0 d/2 -d/2] - (distance/2);
yAnt(1,:) = [2*M/3 -M/3 -M/3];

xAnt(2,:) = [0 d/2 -d/2] + (distance/2);
yAnt(2,:) = [2*M/3 -M/3 -M/3];

% геометрический центр 2 АР
xcAnt(1,:) = mean(xAnt(1,:));
xcAnt(2,:) = mean(xAnt(2,:));

ycAnt(1,:) = mean(yAnt(1,:));
ycAnt(2,:) = mean(yAnt(2,:));

% моделирование сетки
dxy = 5; % шаг по oX и oY
r = 1e3; % ширина и высота сетки

% координаты сетки по осям
xGrid = -r : dxy : r; % х 
yGrid = r : -dxy : -r; % y

% декартовы координаты 
[X, Y] = meshgrid(xGrid, yGrid);

% расстояние от приемных элементов до точек координатной сетки
objR = {};
arrNameRabc = ["ra1", "rb1", "rc1","ra2", "rb2", "rc2"];
for i = 1 : 3
    objR.(arrNameRabc(i)) = hypot(X - xAnt(1,i), Y-yAnt(1,i));
    objR.(arrNameRabc(i + 3)) = hypot(X - xAnt(2,i), Y-yAnt(2,i));
end

% номера отсчетов сетки
pairing = [1 2; 2 3; 3 1; 4 5; 5 6; 6 4];
objI = {};
areNameI = ["I1_1", "I2_1", "I3_1", "I1_2", "I2_2", "I3_2"];
for i = 1 : size(areNameI,2)
    one = pairing(i,1); two = pairing(i,2);
    % задержки на приемных элементах
    Tau = (objR.(arrNameRabc(one)) - objR.(arrNameRabc(two))) / c;
    % индексы отсчетов
    objI.(areNameI(1,i)) = round(Tau * fs + nwin / 2 + 1);
end

% цель
phi = 30; % угол пеленга
rTau = 400; % расстояние от геом центра до цели

% расчет геометрического центра сист АР
xcAnt_sist = (xcAnt(1,:) + xcAnt(2,:)) / 2;
ycAnt_sist = (ycAnt(1,:) + ycAnt(2,:)) / 2;

xTar = xcAnt_sist + rTau * sind(phi); % оХ  
yTar = ycAnt_sist + rTau * cosd(phi); % оY

% цель 
% траектория цели 
x_gr = -r : dxy : r; %oX

% соотношение размерностей массива
length_X_gr = length(x_gr); % длинна массива
%шаг для оси Y
step = (abs(min(yGrid)) + abs(max(yGrid))) / (length_X_gr - 1); % шаг для 

% y_gr = 0.45r : step : r; % oY
y_gr = [];
for i = 1 : length(x_gr)
    y_gr(i) = 0.45 * r;
end


xTarArr = x_gr; % оХ  
yTarArr = y_gr; % оY

% массив координат
Q_aim = zeros(size((objI.I1_1)));
xy_aim_Arr = [];

disp(length_X_gr);

for l = 1 : length_X_gr
    disp(l);
    
    xTar = xTarArr(l);
    yTar = yTarArr(l);
    
    % задержки по времени, для каждого приемного элемента
    tauArr = [];
    for i = 1 : n
        tauArr(i,:) = (hypot(xTar - xAnt(i,:), yTar - yAnt(i,:))...
        - hypot(xTar - xcAnt(i,:), yTar - ycAnt(i,:))) / c;
    end
    
    prc_Q_Arr = [];
    pairing = [1 2; 2 3; 3 1]; % пары для комплексного сопряжения
    
    for k = 1 : n
    tau = tauArr(k,:);
        for j = 1 : size(pairing,1)
            % 1, 2 пара компл. сопр.
            one = pairing(j,1); two = pairing(j,2);

            % требуемый имнтервал времени
            sig_t =  [tau(one) + (0:nwin-1)'*dt tau(two) + (0:nwin-1)'*dt];
            % исходный интервал времени
            sig_t0 = min(sig_t(:)):dt:max(sig_t(:)+dt);
            sig_s0 = randn(size(sig_t0)); % нормально распределенный шум

            % создание шумового сиганала приемного элемента
            sig_s = [];
            for i = 1 : size(sig_t,2)
               sig_s(:,i) = interp1(sig_t0, sig_s0, sig_t(:,i), 'spline');
            end

            prc_S = fft(sig_s, [], 1); % БПФ
            prc_X = prc_S(:,2).*conj(prc_S(:,1)); % Комплексное сопряжение
            prc_Q = ifftshift(ifft(prc_X./abs(prc_X))); % ОБПФ

            prc_Q_Arr = [prc_Q_Arr; prc_Q']; 
        end
    end
    
    % фильтрация данных
    prc_Q_Arr = firstFilter(prc_Q_Arr);

    prc_Q_sum_1 = zeros(size(objI.I1_1));
    prc_Q_sum_2 = prc_Q_sum_1;
    for i = 1 : size(xAnt,2)
        prc_Q1 = prc_Q_Arr(i,:);
        prc_Q_sum_1 = prc_Q_sum_1 + prc_Q1(objI.(areNameI(i)));

        prc_Q2 = prc_Q_Arr(i+3,:);
        prc_Q_sum_2 = prc_Q_sum_2 + prc_Q2(objI.(areNameI(i+3)));
    end
    
    % повторная фильтрация данных
    prc_Q_sum = secondFilter(prc_Q_sum_1) + secondFilter(prc_Q_sum_2);
    
    % итоговая фильтрация данных % обнаружение цели
    prc_Q_sum = secondFilter(prc_Q_sum);
    
    [M, I] = max(prc_Q_sum(:));
    [I_row, I_col] = ind2sub(size(prc_Q_sum), I);
    xy_aim = [xGrid(I_col) yGrid(I_row)];
    xy_aim_Arr = [xy_aim_Arr; xy_aim];

    % координаты движения цели
    Q_aim = Q_aim + prc_Q_sum;
end
 
Q_aim_filter = medfilt2(Q_aim, [10 10]);
figure(1); clf;
imagesc(xGrid, yGrid, Q_aim);
title('траектория цели без фильтра');
set(gca, 'YDir', 'normal');colormap('parula');

figure(2); clf;
imagesc(xGrid, yGrid, Q_aim_filter);
title('траектория цели c фильтром');
set(gca, 'YDir', 'normal');colormap('parula');

% координаты цели 
xArr = xy_aim_Arr(:,1)';
yArr = xy_aim_Arr(:,2)';

% медиаанная филтьтрация оси x, преобразование массива Y к размеру 
xArrFilter = movmedian(xArr, 100, 'omitnan');

figure(2),
        plot(xcAnt(1), ycAnt(1), 'or', 'LineWidth',2);hold on; grid on;
        plot(xcAnt(2), ycAnt(2), 'ob', 'LineWidth',2);
        plot(xArr, yArr, '-g', 'LineWidth',2);
        plot(xArrFilter, yArr, '-b', 'LineWidth',2);
        xlabel('Расстояние, м'); ylabel('Расстояние, м');
        legend('1 приемник', '2 приемник','ошибка', 'Фильтрация ошибки');
        
% поиск ошибки ось x
xArrowFiltr = abs(xArrFilter) - abs(x_gr);
xArrowNoFiltr = abs(xArr) - abs(x_gr);

% поиск ошибки по оси y
yArrowFiltr = abs(yArr) - abs(y_gr);

figure(3),
    subplot(3,1,1);
    plot(xArrowFiltr); title('Ось х, Есть фильтр');
    subplot(3,1,2);
    plot(xArrowNoFiltr); title('Ось х, Есть фильтр');
    subplot(3,1,3);
    plot(yArrowFiltr); title('Ось y, Нет фильтра');


% код функций
function itog = firstFilter(Arr)
    [str col] = size(Arr);
    for i = 1 : str
        maxArr = max(Arr(i,:));
        for j = 1 : col
            if Arr(i,j) < maxArr
                Arr(i,j) = 0;
            end
        end
    end
    itog = Arr;
end

function itog = secondFilter(Arr)
    maxArr = max(max(Arr));
    [str col] = size(Arr);
    for i = 1 : str
        for j = 1 : col
            if Arr(i,j) < maxArr
                Arr(i,j) = 0;
            end
        end
    end
    itog = Arr;
end





