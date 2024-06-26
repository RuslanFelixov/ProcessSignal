clear; close all; clc;

%% исходные данные о ТПА и цели
% размер поля действия ТПА
sizeField = 1000;

c = 1500; % скорость звука в воде
betta = 20; % раствор ХН антенны ПА
dr = 1000; % дальность обнаружения ТПА
tk = dr * 2 / 1500; % время 1 такта обработки

arrAngleTPA = 0 : betta : 180; % диапазон поворота ТПА

s.coefNoise = 0.5; % коэффициент шума

% данные ТПА и цели
TPA_1.num = 1; 
TPA_1.xStart = 100;
TPA_1.yStart = 600;
TPA_1.speed = 30.87;
TPA_1.kurs = 90;
TPA_1.coef = -1; % направление ХН вниз
TPA_1.findAim = false; % обнаружена ли цель
TPA_1.angleAim = 0; % угол поворота на, котором ТПА обнаружил цель
TPA_1.showXN = false; % если обнаружени цель, то ХН больше не показывать
% траектория ТПА
TPA_1.xArr = [TPA_1.xStart]; TPA_1.yArr = [TPA_1.yStart];

TPA_2.num = 2;
TPA_2.xStart = 100;
TPA_2.yStart = 190;
TPA_2.speed = 30.87;
TPA_2.kurs = 90;
TPA_2.coef = 1; % направление ХН вверх
TPA_2.findAim = false; % обнаружена ли цель
TPA_2.angleAim = 0; % угол поворота на, котором ТПА обнаружил цель
TPA_2.showXN = false; % если обнаружени цель, то ХН больше не показывать
% траектория ТПА
TPA_2.xArr = [TPA_2.xStart]; TPA_2.yArr = [TPA_2.yStart];

AIM.xStart = 500;
AIM.yStart = 400;
AIM.speed = 10.28; 
AIM.kurs = 45; % курсовой угол
% преобразование курсового угла, коэф направления (вверх или вниз)
[AIM.angle, AIM.coef] = transformAngleCoef(AIM.kurs);
% коеф направление (цель отдаляется или приближается)
AIM.direction = getDirection(AIM.kurs);
% траектория цели
AIM.xArr = [AIM.xStart]; AIM.yArr = [AIM.yStart];

% пеленг на цель в самом начале
disp('До обнаружения цели');
disp("Исходный пеленг 1 ТПА " + string(getPelengTPAStart(TPA_1, AIM)));
disp("Исходный пеленг 2 ТПА " + string(getPelengTPAStart(TPA_2, AIM)));

figure, % исходные кординаты цели и ТПА
    plot(TPA_1.xStart, TPA_1.yStart, '*r', 'lineWidth', 2); hold on; 
    plot(TPA_2.xStart, TPA_2.yStart, '*g', 'lineWidth', 2);grid on;
    plot(AIM.xStart, AIM.yStart, '*b', 'lineWidth', 2);
    legend('ТПА 1', 'ТПА 2', 'Цель');
    title('Местоположение ТПА и цели в начеле их поиска');
    xlabel('x, м'); ylabel('y, м');

%% моделирование траектории ТПА и Цели
for i = 1 : length(arrAngleTPA)
    % угол поворота каждого ТПА для поиска цели
    angleTpa_1 = arrAngleTPA(i); 
    angleTpa_2 = arrAngleTPA(i);
    
    % если 1 тпа обнаружил цель, происходит коррекция курса с учетом пеленга 
    if TPA_1.findAim == true || TPA_2.findAim == true
        if arrAngleTPA(i) < 90
            angleTpa_1 = TPA_1.angleAim;
            angleTpa_2 = TPA_2.angleAim;
        elseif arrAngleTPA(i) > 90
            angleTpa_1 = 180 - TPA_1.angleAim; 
            angleTpa_2 = 180 - TPA_2.angleAim;
        end
    end
        
    % координаты ТПА и цели
    [xTPA_1, yTPA_1] = moveTPA(TPA_1, angleTpa_1, tk);
    [xTPA_2, yTPA_2] = moveTPA(TPA_2, angleTpa_2, tk);
    [xAIM, yAIM] = moveAim(AIM, tk);
    
    % массив координат ТПА и Цели
    TPA_1.xArr(end + 1) = xTPA_1; TPA_1.yArr(end + 1) = yTPA_1;
    TPA_2.xArr(end + 1) = xTPA_2; TPA_2.yArr(end + 1) = yTPA_2;
    AIM.xArr(end + 1) = xAIM; AIM.yArr(end + 1) = yAIM;
    
    % координаты ХН ТПА
    [TPA_1.xXN, TPA_1.yXN] = drawXN(TPA_1, angleTpa_1, dr, betta);
    [TPA_2.xXN, TPA_2.yXN] = drawXN(TPA_2, angleTpa_2, dr, betta);
    
    % координаты вершин треугольника ХН
    TPA_1.coordTrain = getCoordTrain(TPA_1, angleTpa_1, dr, betta);
    TPA_2.coordTrain = getCoordTrain(TPA_2, angleTpa_2, dr, betta);
    
    % обнаружил ли цель 1 ТПА
    if TPA_1.findAim == false 
        TPA_1.findAim = pointInTriangle(TPA_1.coordTrain, [xAIM yAIM]); 
        if TPA_1.findAim == true % если 1 ТПА обнаружил цель
            % поправка курса ТПА с учетом пеленга
            [TPA_1.angleAim, TPA_1.coef, TPA_2.angleAim, TPA_2.coef] = ...
                getPelengDetermineTPA(TPA_1, TPA_2, AIM); 
        end
    end
    
    % обнаружил ли цель 2 ТПА
    if TPA_2.findAim == false
        TPA_2.findAim = pointInTriangle(TPA_2.coordTrain, [xAIM yAIM]);
        if TPA_2.findAim == true % если 2 ТПА обнаружил цель
            % поправка курса ТПА с учетом пеленга      
            [TPA_1.angleAim, TPA_1.coef, TPA_2.angleAim, TPA_2.coef] = ...
                getPelengDetermineTPA(TPA_1, TPA_2, AIM);        
        end
    end
    
    
    figure(2); % отрисовка координат и ХН ТПА 1 / 2
        % координаты цели 1 ТПА
        plot(TPA_1.xArr(i), TPA_1.yArr(i), '*r', 'LineWidth', 2); hold on;
%         % если нужна ХН на всех стадиях поиска цели 
%         plot(TPA_1.xXN, TPA_1.yXN, '-r');
        
%         ХН 1 ТПА когда цель обнаружени
        if TPA_1.findAim == true % && TPA_1.showXN == false
            plot(TPA_1.xXN, TPA_1.yXN, '-r');
            TPA_1.showXN = true;
        end 
        
        % если нужна ХН на всех стадиях поиска цели
        plot(TPA_2.xArr(i), TPA_2.yArr(i), '*g', 'LineWidth', 2);
%         plot(TPA_2.xXN, TPA_2.yXN, '-g');
        
        % ХН 1 ТПА когда цель обнаружени
        if TPA_2.findAim == true % && TPA_2.showXN == false
            plot(TPA_2.xXN, TPA_2.yXN, '-g');
            TPA_2.showXN = true;
        end
        
        % отрисовка координат цели
        plot(AIM.xArr(i), AIM.yArr(i), '*b', 'LineWidth', 2);
        grid on; % pause(1);
    
    % запись координат для моделирования, углов поворота ТПА
    if TPA_1.findAim == true && TPA_2.findAim == true
        TPA_1.x = TPA_1.xArr(end); TPA_1.y = TPA_1.yArr(end);
        TPA_2.x = TPA_2.xArr(end); TPA_2.y = TPA_2.yArr(end);
        
        AIM.x = AIM.xArr(end); AIM.y = AIM.yArr(end);
        
        TPA_1.angleAim = angleTpa_1; 
        TPA_2.angleAim = angleTpa_2;  
        break;
    end
end

% plot(TPA_1.xArr(end), TPA_1.yArr(end), '*r', 'LineWidth', 2)
% plot(TPA_1.xArr, TPA_1.yArr, '-r' );
% plot(TPA_2.xArr(end), TPA_2.yArr(end), '*g', 'LineWidth', 2);
% plot(TPA_2.xArr, TPA_2.yArr, '-g');
% plot(AIM.xArr, AIM.yArr, '-b');
% plot(AIM.xArr(end), AIM.yArr(end), '*b', 'LineWidth', 2);
% title('Траектория ТПА и цели, ХН ТПА');
% xlabel('x, м'); ylabel('y, м');

% если цель не обнаружени программа завершается
if TPA_1.findAim == false && TPA_2.findAim == false
    disp('Цель не обнаружена');
    return;
else
    disp('Цель обнаружена');
end
 
figure,
    % местоположение
    plot(TPA_1.x, TPA_1.y, '*r', 'lineWidth', 2); hold on; grid on;
    plot(TPA_2.x, TPA_2.y, '*g', 'lineWidth', 2);
    plot(AIM.x, AIM.y, '*b', 'lineWidth', 2);
    
    % ХН   
    plot(TPA_1.xXN, TPA_1.yXN, '-r');
    plot(TPA_2.xXN, TPA_2.yXN, '-g');
    
    % пеленг
    plot([AIM.x TPA_1.x], [AIM.y TPA_1.y], 'b');
    plot([AIM.x TPA_2.x], [AIM.y TPA_2.y], 'b');
    title('Местоположение ТПА и цели, ХН, пеленг в момент обнаружения цели');
    xlabel('x, м'); ylabel('y, м'); 

%% моделирование, когда цель обнаружена
% пеленг без учета поворота ТПА
TPA_1.peleng = getPelengTPA(TPA_1, AIM);
TPA_2.peleng = getPelengTPA(TPA_2, AIM);
disp("Исходный пеленг 1 ТПА " + string(TPA_1.peleng));
disp("Исходный пеленг 2 ТПА " + string(TPA_2.peleng));

% определение пеленга в момент обнаружения цели c учетом ХН и поворота ТПА
TPA_1.pelengXN = -1 * (TPA_1.angleAim  - getPelengTPA(TPA_1, AIM));
TPA_2.pelengXN = TPA_2.angleAim + getPelengTPA(TPA_2, AIM);

disp("Пеленг цели на антенну ТПА с учетом ХН и Угла поворота");
disp("пеленг 1 ТПА " + string(TPA_1.pelengXN));
disp("пеленг 2 ТПА " + string(TPA_2.pelengXN));

% бистатический угол
B = abs(TPA_1.peleng) + abs(TPA_2.peleng);

% курсовой угол цели для каждого ТПА 
TPA_1.q = AIM.kurs + (90 - TPA_1.peleng);
 
if (TPA_1.q < 0)
    TPA_1.q = abs(TPA_1.q);
    TPA_2.q = B - TPA_1.q;   
else
    TPA_2.q = TPA_1.q + B;
end

%% создание сигнала на ПИ элементов
% исходные данные сигнала
% исходные частоты сигналов 
TPA_1.f0 = 20000;
TPA_2.f0 = 23000;

% частоты дискретизации сигналов
TPA_1.fd = 84440;
TPA_2.fd = 97050;

% полоса фильтра 
TPA_1.workF = 2220;
TPA_2.workF = 2520;

% исходные данные сигнала
s.ts = 0.25;% время генерации сигнала
s.c = 1500; % скорость звука в воде

% время 1 отчета
TPA_1.dt = 1 / TPA_1.fd;
TPA_2.dt = 1 / TPA_2.fd;

% Массив временных отсчётов
TPA_1.t = 0 : TPA_1.dt : s.ts; 
TPA_2.t = 0 : TPA_2.dt : s.ts;

% кол-во отсчетов сигнала
TPA_1.n = size(TPA_1.t, 2);
TPA_2.n = size(TPA_2.t, 2);

% исходные данные антенны
ant.M = 8; % число ПИ антенны

% дублирование строк t c учетом кол-ва ПИ антенны ТПА
TPA_1.T = repmat(TPA_1.t, ant.M, 1);
TPA_2.T = repmat(TPA_2.t, ant.M, 1);

% максимальное расстояние между ПИ антенны d < c / 2 * f
ant.d = s.c / (2 * TPA_2.f0);

% формирование массива задержек на ПИ антенны ТПА
TPA_1.tDelayArr = getArrTimeDalayPI(TPA_1, ant, s);
TPA_2.tDelayArr = getArrTimeDalayPI(TPA_2, ant, s);

% определение доплеровского сдвига частоты 
TPA_1.fdop = getDopplerFrequency(AIM, TPA_1, s);
TPA_2.fdop = getDopplerFrequency(AIM, TPA_2, s);
  
% сформированный сигнал на без добавления шума и временной задержки сигнала
% от тпа до цели (задержка на ПИ)
TPA_1.s0 = getSignal(TPA_1);
TPA_2.s0 = getSignal(TPA_2);
  
% % вывод сгенерированных сигналов сигналов
% viewGraphSignal(TPA_1, ant);
% viewGraphSignal(TPA_2, ant);
% viewGraphSignalImg(TPA_1, ant);
% viewGraphSignalImg(TPA_2, ant);
%  
% % % поиск временной задерки сигнала
TPA_1.delaySig = getTimeDalaySignal(TPA_1, AIM, s);
TPA_2.delaySig = getTimeDalaySignal(TPA_2, AIM, s);
  
% определение времени такта обработки
dr = 1000; % дальность обнаружения ТПА
s.tTact = dr * 2 / s.c; % время 1 такта обработки

TPA_1.tTactProcess = 0 : TPA_1.dt : s.tTact;
TPA_2.tTactProcess = 0 : TPA_2.dt : s.tTact;

% формирование сигналов с учетом временной задержки и шума
TPA_1.sigNoise = getSignalNoise(TPA_1, s, ant);
TPA_2.sigNoise = getSignalNoise(TPA_2, s, ant);

% вывод сгенерированных сигналов сигналов
% viewGraphSignalNoise(TPA_1, ant);
% viewGraphSignalNoise(TPA_2, ant);
% viewGraphSignalNoiseImg(TPA_1, ant);
% viewGraphSignalNoiseImg(TPA_2, ant);

%% обработка сигналов
% % параметры обработки
proc.nfft = 4096; % количество точек бпф

% шаг дискретизации в частотной области
TPA_1.df = TPA_1.fd / proc.nfft;
TPA_2.df = TPA_2.fd / proc.nfft;

% дискретные значения частот
TPA_1.fk = TPA_1.df * (0 : proc.nfft - 1);
TPA_2.fk = TPA_2.df * (0 : proc.nfft - 1);

% частотная полоса анализа
TPA_1.fr = [TPA_1.fk(1 : proc.nfft / 2),  ...
    TPA_1.fk((proc.nfft / 2 + 1) : proc.nfft) - TPA_1.fd];

TPA_2.fr = [TPA_2.fk(1 : proc.nfft / 2),  ...
    TPA_2.fk((proc.nfft / 2 + 1) : proc.nfft) - TPA_2.fd];

% кол-во целы интервалов с учетом кол-ва точек бпф, в кот влезает сигнал
sn_fft_1 = floor(TPA_1.n / proc.nfft);
sn_fft_2 = floor(TPA_2.n / proc.nfft);

order = 12; % порядок каждого полосового фильтра

% параметры полосового фильтра 1 ТПА
filtrPar1.fLow  = TPA_1.f0 - 0.5 * TPA_1.workF; % нижняя частота
filtrPar1.fHigh = TPA_1.f0 + 0.5 * TPA_1.workF; % верхняя частота
filtrPar1.order = order; % порядок
     
% параметры полосового фильтра 2 ТПА
filtrPar2.fLow  = TPA_2.f0 - 0.5 * TPA_2.workF; % нижняя частота
filtrPar2.fHigh = TPA_2.f0 + 0.5 * TPA_2.workF; % верхняя частота
filtrPar2.order = order; % порядок

% полосовая фильтрация сигнала
% Полосовая фильтрация пришедшего сигнала
TPA_1.sigNoise = filterBandPassSignal(TPA_1.sigNoise, filtrPar1, TPA_1.fd, ant.M);
TPA_2.sigNoise = filterBandPassSignal(TPA_2.sigNoise, filtrPar2, TPA_2.fd, ant.M);

% вывод сигналов после полосовой фильтрации
% viewGraphSignalFiltr(TPA_1, ant);
% viewGraphSignalFiltr(TPA_2, ant);
% viewGraphSignalFilterImg(TPA_1, ant);
% viewGraphSignalFilterImg(TPA_2, ant);

% получить пеленг и выделить сигнал на фоне шума
[tpa_1.pelengXN, indTStart_1, indTEnd_1] = ...
    getPelengAndSignal(TPA_1, s, ant, proc, sn_fft_1, filtrPar1);
[tpa_2.pelengXN, indTStart_2, indTEnd_2] = ...
    getPelengAndSignal(TPA_2, s, ant, proc, sn_fft_2, filtrPar2);

tpa_1.indTStart = indTStart_1;
tpa_2.indTStart = indTStart_2;

% выделение сигналов на фоне шумов
tpa_1.s0 = TPA_1.sigNoise(:, indTStart_1 : indTEnd_1);
tpa_2.s0 = TPA_2.sigNoise(:, indTStart_2 : indTEnd_2);
 
% вывод сигналов
% viewSearchSignal(TPA_1, s, ant, indTStart_1, indTEnd_1);
% viewSearchSignal(TPA_2, s, ant, indTStart_2, indTEnd_2);

tpa_1.num = 1; tpa_2.num = 2;

% вывод сигналов после вырезания шума
% viewGraphRealSignal(tpa_1, TPA_1, ant);
% viewGraphRealSignal(tpa_2, TPA_2, ant);
% viewGraphRealSignalImg(tpa_1, TPA_1, ant);
% viewGraphRealSignalImg(tpa_2, TPA_2, ant);

disp("Пеленг обнаруженного сигнала");
disp("1 ТПА (ХН)" + string(tpa_1.pelengXN));
disp("2 ТПА (ХН)" + string(tpa_2.pelengXN));

% перерасчет пеленга с учетом угла поворота ТПА
tpa_1.peleng = -1 * TPA_1.coef * TPA_1.angleAim + tpa_1.pelengXN;
tpa_2.peleng = -1 * TPA_2.coef * TPA_2.angleAim + tpa_2.pelengXN;
disp("Пеленг с учетом вектора движения ТПА");
disp("1 ТПА " + string(tpa_1.peleng));
disp("2 ТПА " + string(tpa_2.peleng));

% % % определение сдвига частоты сигнала
tpa_1.fdop = getFrequencySignal(tpa_1, s, ant, proc, TPA_1) - TPA_1.f0;
tpa_2.fdop = getFrequencySignal(tpa_2, s, ant, proc, TPA_2) - TPA_2.f0;

% % бистатический угл
BB = abs(tpa_1.peleng) + abs(tpa_2.peleng);
%  
% % % Определение  радиальной скорости цели относительно каждого ТПА
tpa_1.radVAim = getRadialSpeedAim(tpa_1, TPA_1, s);
tpa_2.radVAim = getRadialSpeedAim(tpa_2, TPA_2, s);
%  
% % % Определение курсового угла цели для каждого ТПА
tpa_1.q = getKursAngleAim(tpa_1.radVAim, tpa_2.radVAim, BB, 1);
tpa_2.q = getKursAngleAim(tpa_1.radVAim, tpa_2.radVAim, BB, 2);
%  
% % определение абсолютной скорости цели
tpa_1.vAim1 = getSpeedAim(tpa_1);
tpa_2.vAim2 = getSpeedAim(tpa_2);
% 
% % определение истинного курса цели
tpa_1.Kc = TPA_1.kurs + tpa_1.peleng - (180 - tpa_1.q);
tpa_2.Kc = TPA_2.kurs + tpa_2.peleng - (180 - tpa_2.q);
%  
% % определение погрешностей
% % % Определение погрешности частоты
deltaFdop1 = abs(TPA_1.fdop - tpa_1.fdop);
deltaFdop2 = abs(TPA_2.fdop - tpa_2.fdop);
% 
disp("Погрешность доплеровского сдвига 1 ТПА = " + string(deltaFdop1));
disp("Погрешность доплеровского сдвига 2 ТПА = " + string(deltaFdop2));
% 
% % % Определение погрешности частоты
deltaPeleng1 = abs(TPA_1.peleng - tpa_1.peleng);
deltaPeleng2 = abs(TPA_2.peleng - tpa_2.peleng);
% 
disp("Погрешность пеленга 1 ТПА = " + string(deltaPeleng1));
disp("Погрешность пеленга 2 ТПА = " + string(deltaPeleng2));
% 
% % % Определение погрешности радиальной скорости
radVAim1 = abs(getRadialSpeedAimStart(TPA_1, s) - tpa_1.radVAim);
radVAim2 = abs(getRadialSpeedAimStart(TPA_2, s) - tpa_2.radVAim);
% 
disp("Погрешность радиальной скорости 1 ТПА = " + string(radVAim1));
disp("Погрешность радиальной скорости 2 ТПА = " + string(radVAim2));
% 
% % % определение погрешности абсолютной скорости
deltaspeed = abs(AIM.speed - tpa_1.vAim1);
% 
disp("Погрешность абсолютной скорости = " + string(deltaspeed));
% % 
% % определение погрешности Курса движения цели
deltaKurs = abs(abs(AIM.kurs) - abs(tpa_1.Kc ));
% 
disp("Погрешность Курса цели = " + string(deltaKurs));
% 
% % погрешность бистатического угла
deltaB = abs(BB - B);
disp("Погрешность бистатического угла " + string(deltaB));
% 
% % % определение погрешности в определениии местоположении цели
tpa_1.x = TPA_1.x;
tpa_1.y = TPA_1.y;

tpa_2.x = TPA_2.x;
tpa_2.y = TPA_2.y;

[xAim, yAim] = findCoordAim(tpa_1, tpa_2, dr);
deltaAimX = abs(AIM.x - xAim);
deltaAimY = abs(AIM.y - yAim);

% определение с помощью проекции пеленга
disp("Определение координат с помощью проекций плеленга");
disp("Погрешность определения координат цели  по оХ = " + string(deltaAimX) + ...
    "м по оY = " + string(deltaAimY) + " м");
 
figure, 
    plot(TPA_1.x, TPA_1.y, '*r', 'lineWidth', 2); hold on; grid on;
    plot(TPA_2.x, TPA_2.y, '*g', 'lineWidth', 2);
    plot(AIM.x, AIM.y, '*b', 'lineWidth', 2);
    plot(xAim, yAim, '*m', 'lineWidth', 2);
    legend('ТПА 1', 'ТПА 1', 'Цель', 'Цель рассчет');
%     xlim([0, sizeField]); ylim([0, sizeField]);
    title('Определение местоположения и цели c помощью проекций пленга');
    
[xAim1, yAim1] = getCoordWithHelpTime(TPA_1, tpa_1, s);
[xAim2, yAim2] = getCoordWithHelpTime(TPA_2, tpa_2, s);

xAim = (xAim1 + xAim2) / 2; yAim = (yAim1 + yAim2) / 2;
deltaAimX = abs(AIM.x - xAim);
deltaAimY = abs(AIM.y - yAim);


disp("Определение координат с помощью проекций плеленга и времени прихода сигнала");
disp("Погрешность определения координат цели  по оХ = " + string(deltaAimX) + ...
    "м по оY = " + string(deltaAimY) + " м");

 
figure, 
    plot(TPA_1.x, TPA_1.y, '*r', 'lineWidth', 2); hold on; grid on;
    plot(TPA_2.x, TPA_2.y, '*g', 'lineWidth', 2);
    plot(AIM.x, AIM.y, '*b', 'lineWidth', 2);
    plot(xAim, yAim, '*m', 'lineWidth', 2);
    legend('ТПА 1', 'ТПА 1', 'Цель', 'Цель рассчет');
%     xlim([0, sizeField]); ylim([0, sizeField]);
    title('Определение местоположения и цели с помощью времени прихода сигнала и проекций пеленга');

% вывод графиков на ПИ ТПА
function viewGraphSignalFiltr(TPA, ant)
    figure,
    for i = 1 : ant.M
        subplot(ant.M, 1, i);
        plot(TPA.tTactProcess, TPA.sigNoise(i,:));
        xlabel('t, c'); ylabel('S(t)');
        nameGraph = string(i) + " ПИ";
        title(nameGraph); grid on;
    end

    nameGroupGraph = "Осциллограмма сигналов после ПФ на ПЭ ТПА " + string(TPA.num);
    suptitle(nameGroupGraph);     
end

% вывод графиков на ПИ ТПА
function viewGraphSignalFilterImg(TPA, ant)
    yAxis = 1 : ant.M;

    figure,
        imagesc(TPA.tTactProcess, yAxis, TPA.sigNoise);
        xlabel('t, c'); ylabel('№ ПЭ');
        nameGraph = "Яркостное представление сигналов после ПФ на ПЭ ТПА ";
        title(nameGraph  + string(TPA.num));     
end

function [x, y] = getCoordWithHelpTime(TPA,tpa, s)
    dr = 0.5 * TPA.tTactProcess(tpa.indTStart) * s.c;

    x = tpa.x + dr * cosd(tpa.peleng);
    y = tpa.y - dr * sind(tpa.peleng); 
end

%% Функции траекторий ТПА и цели
function [angle_1, coef_1, angle_2, coef_2] = ...
    getPelengDetermineTPA(TPA_1, TPA_2, AIM)
    % пеленг цели на ТПА
    angle_1 = round(getAngleRotation ...
        (TPA_1.xArr, TPA_1.yArr, AIM.xArr, AIM.yArr));
            
    angle_2 = round(getAngleRotation ...
        (TPA_2.xArr, TPA_2.yArr, AIM.xArr, AIM.yArr));
              
    % определить выше или ниже находится цель относительно ТПА
    coef_1 = determinePositionTPA(TPA_1.yArr, AIM.yArr);
    coef_2 = determinePositionTPA(TPA_2.yArr, AIM.yArr);
end

function coef = determinePositionTPA(yTPA, yAIM)
    if yTPA(end) > yAIM(end)
        coef = -1; % направление ТПА вниз
    else
        coef = 1; % направление ТПА вверх
    end    
end

function angle = getAngleRotation(tpaX, tpaY, aimX, aimY)
    % длина катета
    lengthX = abs(tpaX(end) - aimX(end)); % Х
    lengthY = abs(tpaY(end) - aimY(end)); % Y
    angle = atand(lengthY / lengthX); 
end

function isInside = pointInTriangle(coordTrain, aim)    
    % Разбиваем треугольник на три подтреугольника
    triangle1 = [coordTrain(1,:); coordTrain(2,:); aim];
    triangle2 = [coordTrain(2,:); coordTrain(3,:); aim];
    triangle3 = [coordTrain(3,:); coordTrain(1,:); aim];
    
    % Вычисляем площади треугольников
    totalArea = polyarea(coordTrain(:,1), coordTrain(:,2));
    area1 = polyarea(triangle1(:,1), triangle1(:,2));
    area2 = polyarea(triangle2(:,1), triangle2(:,2));
    area3 = polyarea(triangle3(:,1), triangle3(:,2));
    
    % лежит ли точка внутри треугольника
    isInside = (round(totalArea) == round(area1 + area2 + area3));
end

function coordTrain = getCoordTrain(TPA, angle, dr, betta)
    xEnd = TPA.xArr(end); yEnd = TPA.yArr(end);

    l = dr / cosd(betta/2);
    angleTPA = TPA.coef * angle;
    
    X_dr_r = xEnd + l * sind(90 - angleTPA - betta/2);
    Y_dr_r = yEnd + l * cosd(90 - angleTPA - betta/2);

    X_dr_l = xEnd + l * sind(90 - angleTPA + betta/2);
    Y_dr_l = yEnd + l * cosd(90 - angleTPA + betta/2);
      
    coordTrain = [xEnd yEnd; X_dr_r Y_dr_r; X_dr_l Y_dr_l];
end

function [x, y] = moveAim(obj, tk)
    STPA = obj.speed * tk;
    x = obj.xArr(end) + obj.direction * STPA * cosd(obj.angle);
    y = obj.yArr(end) + obj.coef * STPA * sind(obj.angle); 
end

function [X_XN, Y_XN] = drawXN(TPA, angle, dr, betta)
    xEnd = TPA.xArr(end); yEnd = TPA.yArr(end);
    
    angleTPA = TPA.coef * angle; 
    
    X_dr_c = xEnd + dr * cosd(angleTPA);
    Y_dr_c = yEnd + dr * sind(angleTPA);

    l = dr / cosd(betta/2);
    
    X_dr_r = xEnd + l * sind(90 - angleTPA - betta/2);
    Y_dr_r = yEnd + l * cosd(90 - angleTPA - betta/2);

    X_dr_l = xEnd + l * sind(90 - angleTPA + betta/2);
    Y_dr_l = yEnd + l * cosd(90 - angleTPA + betta/2);

    X_XN = [xEnd X_dr_r X_dr_l xEnd X_dr_c];
    Y_XN = [yEnd Y_dr_r Y_dr_l yEnd Y_dr_c];
end

function [x, y] = moveTPA(tpa, angle, tk)
    STPA = tpa.speed * tk;
    
    x = tpa.xArr(end) + STPA * cosd(angle);
    y = tpa.yArr(end) + tpa.coef * STPA * sind(angle); 
end

function direct = getDirection(kurs)
    if kurs > 0
        direct = 1;
    elseif kurs < 0
        direct = -1;
    else 
        direct = 1;
    end
end

function [angle, coef] = transformAngleCoef(kurs)
    if abs(kurs) < 90
        angle = -1 * (90 - abs(kurs));
        coef = -1; 
    elseif abs(kurs) >= 90
        angle = (abs(kurs) - 90);
        coef = 1;
    end
end
       
%% Функции для расчета
function [X, Y] = findCoordAim(tpa1, tpa2, dr)
    % исходные координаты
    x1 = tpa1.x; y1 = tpa1.y;
    x2 = x1 + dr * cosd(tpa1.peleng);
    y2 = y1 - dr * sind(tpa1.peleng);
    
    x3 = tpa2.x; y3 = tpa2.y;
    x4 = x3 + dr * cosd(tpa2.peleng);
    y4 = y3 -  dr * sind(tpa2.peleng);
    
    % Уравнения прямых для отрезков 
    m1 = (y2 - y1) / (x2 - x1);
    c1 = y1 - m1 * x1;
    
    m2 = (y4 - y3) / (x4 - x3);
    c2 = y3 - m2 * x3;

    % Находим точку пересечения
    X = (c2 - c1) / (m1 - m2);
    Y = m1 * X + c1;
%     
%     figure, 
%         plot([x1, x2], [y1, y2], 'r'); hold on; grid on;
%         plot([x3, x4], [y3, y4], 'g');
%         plot(X, Y, 'b*', 'lineWidth', 2);
%         legend('ТПА 1', 'ТПА 2', 'Цель');
%         title('Определение местоположения Цели');
%         xlabel('x, м'); ylabel('y, м');   
end

% определение скорости цели
function vAim = getSpeedAim(tpa)
    vAim = tpa.radVAim / cosd(tpa.q);
end

% Определение курсового угла цели для каждого ТПА
function q = getKursAngleAim(radVAim_1, radVAim_2, BB, numTpa) 
    if numTpa == 1
        spedNomirator_1 = radVAim_1;
        spedNomirator_2 = radVAim_2;
        spedDenominator = radVAim_1;
    elseif numTpa == 2
        spedNomirator_1 = radVAim_2;
        spedNomirator_2 = radVAim_1;
        spedDenominator = radVAim_2;
    end
    
    numerator = spedNomirator_1 * cosd(BB) - spedNomirator_2;
    denominator =  spedDenominator * sind(BB);
    
    q = atan2(numerator, denominator) * 180 / pi;
end

% Определение  радиальной скорости цели относительно каждого ТПА
function radAim = getRadialSpeedAimStart(TPA, s)
    radAim = 0.5 * TPA.fdop * s.c / TPA.f0 - TPA.speed * cosd(TPA.peleng);
end

function radAim = getRadialSpeedAim(tpa, TPA, s)
    radAim = 0.5 * tpa.fdop * s.c / TPA.f0 - TPA.speed * cosd(tpa.peleng);
end

% определение частоты сигнала 
function frequency = getFrequencySignal(tpa, s, ant, proc, TPA)
    sF=fft(tpa.s0, proc.nfft, 2); % бпф 
    T0 = ant.d * sind(tpa.peleng) / s.c; % задержка 
    Tcomp = T0 * (ant.M - 1 : -1 : 0); % расчет задержек
    coef = exp(1j * 2 * pi * Tcomp' * TPA.fr); % фазирующие коэффициенты
    sComp = sF .* coef; % компенсация временных задержек на ПИ
    PK = abs(sum(sComp, 1)); %формирование ПК 

%     figure, % спектр 2 зон
%         plot(TPA.fr,PK); title('Спектр ПК'); grid on;
%         xlabel('Частота, гц'); ylabel('S(f)');

    frG=TPA.fk(1:proc.nfft/2); % 1 зона найквиста
    sFHalg = zeros(1, proc.nfft/2); % спектр 1 зоны * 2
    sFHalg(1 : proc.nfft / 2) = 2 * PK(1:proc.nfft/2);
    
   figure, 
        plot(frG, sFHalg, 'lineWidth', 2); grid on;
        title("Спектр ПК ТПА " + string(tpa.num));
        xlabel('Частота, гц'); ylabel('S(f)');
    
    [~, maxInd] = max(sFHalg);% определение частоты сигнала
    frequency = frG(maxInd);
end

function viewGraphRealSignalImg(tpa, TPA, ant)
    lengthSignal = size(tpa.s0(1,:), 2) - 1; % кол-во отчетов сигнала
    
    t = 0 : TPA.dt : lengthSignal * TPA.dt;% временные отчеты сигнала
    axisY = 1 : ant.M;
    
    figure, 
        imagesc(t, axisY, tpa.s0);
        xlabel('t, c'); ylabel('№ ПЭ');
        nameTitle = "Яркостное представление фильтрованных сигналов на ТПА ";
        title(nameTitle + string(tpa.num));   
end

% вывод графиков на ПИ ТПА
function viewGraphRealSignal(tpa, TPA, ant)
    lengthSignal = size(tpa.s0(1,:), 2) - 1; % кол-во отчетов сигнала
    t = 0 : TPA.dt : lengthSignal * TPA.dt;% временные отчеты сигнала

    figure,
    for i = 1 : ant.M
        subplot(ant.M, 1, i);
        plot(t, tpa.s0(i,:)); grid on;
        xlabel('t, c'); ylabel('S(t)');
        nameGraph = string(i) + " ПЭ";
        title(nameGraph);
    end
    
    nameGroupGraph = "Обнаруженный сигнал на ПЭ ТПА ";
    suptitle(nameGroupGraph + string(tpa.num));     
end

function viewSearchSignal(TPA, s, ant, indTStart, indTEnd)
    t = TPA.tTactProcess;
    tS = t(indTStart:indTEnd);
    S1 = TPA.sigNoise;
    S2 = S1(:, indTStart : indTEnd);
    
    figure,
    for i = 1 : ant.M
        subplot(ant.M, 1, i);
            plot(t, S1(1, :)); grid on; hold on;
            plot(tS, S2(1, :));
            xlabel('t, c'); ylabel('S(t)');
            nameGraph = string(i) + " ПЭ";
            title(nameGraph);
    end
    
    nameGraph = "Обнаруженный полезный сигнал на фоне шума на ПЭ ТПР ";
    suptitle(nameGraph + string(TPA.num));   
end

% получить пеленг и сигнал
function [peleng, indTStart, indTEnd] = getPelengAndSignal(TPA, s, ant, proc, sn_fft, filtrPar)
    % индекс обработки полосового фильтра
    indProc.Low=round(filtrPar.fLow / TPA.df + 1); % нижней частоты
    indProc.High=round(filtrPar.fHigh / TPA.df + 1); % верхней частоты
    indProc.Ntact = floor(length(TPA.tTactProcess) / proc.nfft); % расчет числа тактов

    % получить набор откриков
    fi = -20 : 0.5 : 20; % углы фазирования
    S = TPA.sigNoise;
    responseTact = getResponseTact(S, s, ant, proc, indProc, fi, TPA);
    viewResponseTact(fi, responseTact, TPA); % вывод набора откликов
   
    % определение угла фазирования
    [~, indPeleng] = max(sum(responseTact));
    peleng = fi(indPeleng);
    viewResponseTactAngle(fi, responseTact, TPA); % вывод набора откликов
    
    % определение интервала сигнала
    % интервалы обработки сигнала
    tTacts = TPA.tTactProcess(1 : proc.nfft : proc.nfft * indProc.Ntact);
    responseTime = responseTact(:, indPeleng); 
    [indTStart, indTEnd] = ...
        getTimeIntervalSignal(responseTime, proc.nfft, sn_fft);
    viewResponseTactTime(tTacts, responseTime, TPA) % вывод набора откликов
    
    
    function viewResponseTactTime(tTacts, responseTime, TPA)
        figure,
            plot(tTacts, responseTime); grid on;
            title("Набор откликов во времени  на ТПА " ...
                + string(TPA.num));
            xlabel('Время, с');
    end

    function viewResponseTactAngle(fi, responseTact, TPA)
        figure,
            plot(fi, sum(responseTact)); grid on;
            title("Набор откликов по углам фазирвания на ТПА " ...
                + string(TPA.num));
            xlabel('Угол фазирования, φ');
    end
        
    function viewResponseTact(fi, responseTact, TPA)
        figure,
            imagesc(fi, TPA.tTactProcess, responseTact);
            ylabel('t, c'); xlabel('Угол фазирования, φ');
            title("Набор откликов на ТПА " + string(TPA.num));
    end
end

% полосовая фильтрация сигнала
function res = filterBandPassSignal(sigNoise, filtrPar, fd, M)
    % коэффициенты фильтра
    [b, a] = butter(filtrPar.order, [filtrPar.fLow, filtrPar.fHigh] ...
        / (fd / 2), 'bandpass');
    
    sFiltr = zeros(size(sigNoise));
    
    for i = 1 : M
        sFiltr(i,:) = filter(b, a, sigNoise(i,:));
    end
    
    res = sFiltr;
end

function resTact = getResponseTact(SF, s, ant, proc, indProc, fi, TPA)
    responseTact = zeros(indProc.Ntact, length(fi));

    for i = 1 : indProc.Ntact
        iStart= (i - 1) * proc.nfft + 1; % начальный индекс временного отчета
        iEnd = i * proc.nfft; % конечный индекс временного отчета
        sPart=fft(SF(:,iStart:iEnd), proc.nfft, 2); % бпф части сигнала

        for j =  1 : length(fi) % цикл по углам фазирования
            T0 = ant.d * sind(fi(j)) / s.c; % временная задержка угла 
            Tcomp = T0 * (ant.M - 1 : -1 : 0); % расчет временных задержек
            coef = exp(1j*2*pi*Tcomp'*TPA.fr); % фазирующий коэффициент
            sComp=sPart.*coef; % компенсация временных задержек на ПИ
            PK=sum(sComp,1); % формирование ПК в направлении b
            responseTact(i,j)=sum(abs(PK(indProc.Low:indProc.High)).^2,2);%отклик в полосе.
        end
    end
    
    resTact = responseTact;
end


% определение временного интервала сигнала
function [indStart, indEnd] = getTimeIntervalSignal(responseTime, nfft, sn_fft)  
    % Сортируем элементы по убыванию
    % sn_fft элемента с учетом времени сигнала 
    [~, sortInd] = sort(responseTime, 'descend'); 
    indTimeFree = sortInd(1 : sn_fft);

    indStart = (min(indTimeFree) - 1) * nfft + 1;
    indEnd = max(indTimeFree) * nfft;
end

% вывод графиков на ПИ ТПА
function viewGraphSignalNoise(TPA, ant)
    figure,
    for i = 1 : ant.M
        subplot(ant.M, 1, i);
        plot(TPA.tTactProcess, TPA.sigNoise(i,:));
        xlabel('t, c'); ylabel('S(t)');
        nameGraph = string(i) + " ПИ";
        title(nameGraph); grid on;
    end

    nameGroupGraph = "Осциллограмма сигналов на ПЭ ТПА " + string(TPA.num);
    suptitle(nameGroupGraph);     
end

% вывод графиков на ПИ ТПА
function viewGraphSignalNoiseImg(TPA, ant)
    yAxis = 1 : ant.M;

    figure,
        imagesc(TPA.tTactProcess, yAxis, TPA.sigNoise);
        xlabel('t, c'); ylabel('№ ПЭ');
        nameGraph = "Яркостное представление сигналов на ПЭ ТПА ";
        title(nameGraph  + string(TPA.num));     
end

% формирование сигнала пришедшего на ПИ с учетом временной задержки и шума
function signal = getSignalNoise(TPA, s, ant)
    % формирование широкополосной помехи
    filtrPar.order = 2; filtrPar.fLow = 100; filtrPar.fHigh = 50000;
    fd = 200000;

    % формирование сигнал + шум
    noise = s.coefNoise * randn(size(TPA.s0));
    noise = filterBandPassSignal(noise, filtrPar, fd, ant.M);
    
    % вывод графика спектра
%     getSpectrNoise(noise(1,:));
    
    sigNoise = TPA.s0 + noise;
    
    % начальный и конечный отсчет сигнала
    numStart = round(TPA.delaySig / TPA.dt); % начальный отсчет сигнала
    numEnd = numStart + TPA.n - 1; % конечный отсчет сигнала

    % шум до сигнала % шум после сигнала
    noiseStart = s.coefNoise * randn(ant.M, numStart - 1);
    noiseStart = filterBandPassSignal(noiseStart, filtrPar, fd, ant.M);
    
    noiseEnd = s.coefNoise * randn(ant.M, length(TPA.tTactProcess) - numEnd);
    noiseEnd = filterBandPassSignal(noiseEnd, filtrPar, fd, ant.M);
    
    % определить отношение сигнал шум
    NoiseAll = [noiseStart, noise, noiseEnd];
    
    SNR = getSignalToNoise(sum(TPA.s0), sum(NoiseAll));
    disp("SNR " + string(SNR));

    % итоговый сигнал с учетом всех факторов
    signal = [noiseStart, sigNoise, noiseEnd];
end

% получить отношение сигнал шум
function SNR = getSignalToNoise(signal, noise)
    % Мощность
    
    sPower = sum(signal.^2) / length(signal); % сигнала
    noisePower = sum(noise.^2) / length(noise); %  шума
    
%     sPower = sqrt(sum(signal.^2) / length(signal)); % сигнала
%     noisePower = sqrt(sum(noise.^2) / length(noise)); %  шума
    
    SNR = noisePower / sPower; % отношение мощностей сигнала и шума
end

function getSpectrNoise(noise)
    fd = 200000; % частота дискретизации
    nfft = 4096; % количество точек бпф
    df = fd / nfft; % шаг дискретизации в частотной области 
    fk = df * (0 : nfft - 1); % дискретные значения частот

    sF=fft(noise, nfft); % бпф 

    frG = fk(1 : nfft/2); % 1 зона найквиста
    sFHalg = zeros(1, nfft / 2); % спектр 1 зоны * 2
    sFHalg(1 : nfft / 2) = 2 * sF(1:nfft/2);
    
   figure, 
        plot(frG, abs(sFHalg)); grid on;
        title("Спектр шумовой помехи");
        xlabel('Частота, гц'); ylabel('N(f)');
end

% определение времени такта обработки 
function tTact = getTimeTactProcessSig(sizeField, s)
    lengthHypotenuse = sqrt(sizeField^2 + sizeField^2);
    tTact = lengthHypotenuse * 2 / s.c;
end

% Определение время пути сигнала
function delay = getTimeDalaySignal(TPA_1, AIM, s)
    lengthX = abs(TPA_1.x - AIM.x); % длина катета Х 
    lengthY = abs(TPA_1.y - AIM.y); % длина катета Y
    % гипотенуза
    lengthHypotenuse = sqrt(lengthX^2 + lengthY^2);
    delay = lengthHypotenuse * 2 / s.c;
end

% вывод графиков на ПИ ТПА
function viewGraphSignal(TPA, ant)
    figure,
    for i = 1 : ant.M
        subplot(ant.M, 1, i);
        plot(TPA.t, TPA.s0(i,:));
        xlabel('t, c'); ylabel('S(t)');
        nameGraph = string(i) + " ПИ";
        title(nameGraph); grid on;
    end

    nameGroupGraph = "Осциллограмма сигналов на ПЭ ТПА " + string(TPA.num);
    suptitle(nameGroupGraph);     
end

function viewGraphSignalImg(TPA, ant)
    axisY = 1 : ant.M;
    
    figure, 
        imagesc(TPA.t, axisY, TPA.s0);
        xlabel('t, c'); ylabel('№ ПЭ');
        nameTitle = "Яркостное представление сигналов на ТПА ";
        title(nameTitle + string(TPA.num));
end

% формирование сигнал на ПИ
function S = getSignal(TPA)
    % формированный сигнал на ПИ
    S = sin(2 * pi * (TPA.f0 + TPA.fdop) * (TPA.T - TPA.tDelayArr));
end

% Определение массива временных задержек на ПИ антенны
function tDelayArr = getArrTimeDalayPI(TPA, ant, s)
    tDelay = ant.d * sind(TPA.pelengXN) / s.c; % 1 задержека
    tDelayArr = (ant.M - 1 : -1 : 0)' * tDelay;
end

% Определение доплеровской частоты
function fdop = getDopplerFrequency(AIM, TPA, s)
    numerator = AIM.speed * cosd(TPA.q) + TPA.speed * cosd(TPA.peleng);
    fdop = 2 * TPA.f0 * numerator / s.c;
end
   
% Определение пеленга на ТПА
function peleng = getPelengTPA(TPA, AIM)
    lengthX = abs(TPA.x - AIM.x); % длина катета Х
    lengthY = abs(TPA.y - AIM.y); % длина катета Y
    
    if TPA.x == AIM.x
        if  TPA.y > AIM.y
            peleng = -90;
        end
     
        if  TPA.y < AIM.y
            peleng = 90;
        end
    else
        if TPA.y > AIM.y
            peleng = atand(lengthY / lengthX); 
        else
            peleng = -1 * atand(lengthY / lengthX);
        end
    end
end


% Определение пеленга на ТПА
function peleng = getPelengTPAStart(TPA, AIM)
    lengthX = abs(TPA.xStart - AIM.xStart); % длина катета Х
    lengthY = abs(TPA.yStart - AIM.yStart); % длина катета Y
    
    if TPA.xStart == AIM.xStart
        if  TPA.yStart > AIM.yStart
            peleng = -90;
        end
     
        if  TPA.yStart < AIM.yStart
            peleng = 90;
        end
    else
        if TPA.yStart > AIM.yStart
            peleng = atand(lengthY / lengthX); 
        else
            peleng = -1 * atand(lengthY / lengthX);
        end
    end
end


    
    
    
