% Definimos los parámetros
N = 100; % Cantidad de bits a generar
M = 2; % Cantidad de símbolos de la modulación (en este caso, BPSK)
k = log2(M); % Cantidad de bits por símbolo

% Generamos los bits aleatoriamente
bits = randi([0 1], 1, N);

% Hacemos un reshape para obtener una matriz k x N/k
bits_matriz = reshape(bits, k, N/k);

% Convertimos la matriz de bits a una matriz de enteros
bits_enteros = bit2int(bits_matriz.', 'left-msb');

% Indexamos el array de símbolos para obtener los símbolos a transmitir
simbolos = array_simbolos(simbolos_index);

% Mostramos los primeros 10 bits y los primeros 5 símbolos para verificar
disp(['Bits: ' num2str(bits(1:10))]);
disp(['Símbolos: ' num2str(simbolos(1:5))]);




%CHAT

% Crea el array con los símbolos BPSK
theta = 0; % define el ángulo de fase inicial

%ESTA LA S BIEN????
S = [exp(1i*theta) -exp(1i*theta)];

% Usa el vector ind para indexar el array S y obtener la señal modulada Z
Z = S(ind+1);
disp(Z);


% Define los valores de Eb/N0 en decibelios
EbN0dB = [0]; 

% Calcula la energía promedio por bit de la modulación BPSK
Eb = mean(abs(S).^2);

% Crea un vector vacío para guardar la señal recibida
R = [];

% Recorre los valores de Eb/N0 en decibelios
for x = EbN0dB
    
    % Convierte el valor de Eb/N0 en decibelios a una razón lineal
    EbN0 = 10^(x/10);
    
    % Calcula el valor de N0
    N0 = Eb/EbN0;
    
    % Genera el vector de ruido Gaussiano con media 0 y varianza N0/2
    noise = sqrt(N0/2)*randn(size(Z));
    
    % Suma el ruido a la señal Z y guarda el resultado en R
    R = [R Z + noise];
end

disp(' ');
disp('RUIDO =');
disp([R]);

% Crea una matriz vacía para guardar los bits demodulados
D = [];

% Recorre las columnas de la señal R
for i = 1:length(EbN0dB)
    
    % Extrae la columna i de R
    r = R(:,i);
    
    % Calcula las distancias euclídeas entre cada valor de r y los símbolos BPSK
    dist = abs(repmat(r,1,M) - repmat(S,N/k,1));
    
    % Encuentra el índice del símbolo más cercano para cada valor de r
    [~,idx] = min(dist,[],2);
    
    % Convierte el índice a un valor binario
    d = int2bit(idx-1,k);
    
    % Convierte la matriz d en un vector
    d = reshape(d',1,N);
    
    % Guarda el vector d en D
    D = [D; d];
end

disp(D);

% Crea un vector vacío para guardar la BER
BER = [];

% Recorre las filas de la matriz D
for i = 1:length(EbN0dB)
    
    % Extrae la fila i de D
    d = D(i,:);
    
    % Calcula los errores cometidos al comparar d con X
    errors = sum(d ~= bits_reshape);
    
    % Calcula la BER como el cociente entre los errores y el número total de bits
    ber = errors/N;
    
    % Guarda la BER en el vector BER
    BER = [BER ber];
end


% Crea una figura para el gráfico
figure;

% Representa la curva de BER frente a Eb/N0 en decibelios usando una escala logarítmica en el eje Y
semilogy(EbN0dB,BER,'-o');

% Añade etiquetas a los ejes y un título al gráfico
xlabel('Eb/N0 (dB)');
ylabel('BER');
title('Curva de BER para modulación BPSK');





% Definir el ancho de banda del canal en Hz
B = 1e6;

% Definir un rango de valores para Eb/N0 en dB
EbN0dB = linspace(-10, 20, 100);

% Calcular el valor correspondiente de Eb/N0
EbN0 = 10.^(EbN0dB/10);

% Calcular la relación señal a ruido (SNR) en el receptor
SNR = EbN0 * log2(2);

% Calcular la capacidad máxima del canal en bps
C = B * log2(1 + SNR);

% Dibujar la curva de capacidad frente a Eb/N0(dB)
plot(EbN0dB,C/1e6);
xlabel('Eb/N0(dB)');
ylabel('Capacidad (Mbps)');
grid on;

