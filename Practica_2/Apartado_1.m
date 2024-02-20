clear all;


%QPSK Gray Mapping
M = 4;
Ep=2; phi=(2*pi/M)*(0:M-1);
TC_qpsk=[1 1*1j -1*1j -1];

%GENERACION DE BITS
% Generar un vector de bits aleatorios de longitud 6
bits_b = randi([0, 1], 1, 20000);
disp(bits_b);


M_b = length(TC_qpsk); %simbolos de modulacion
k_b = log2(M_b);   % Número de bits por símbolo
N_b = length(bits_b); % Número total de bits



%CODIFICACION
%n es el numero de bits que se añaden por cada 
%k que es el bit fonte
R = 3;
n_k = R/1;
bits_codificados = repmat(bits_b, R, 1);
bits_codificados = reshape(bits_codificados, 1, []);
disp(bits_codificados);

br = bits_codificados.';

%MODULACION
%bits_reshape = reshape(bits, k, N/k);
bits_reshape_b = reshape(bits_codificados, k_b, []);
disp(bits_reshape_b)
ind_b = bit2int(bits_reshape_b,k_b); % convierte cada columna de X en un entero
simbM_b=TC_qpsk(ind_b+1);
disp(simbM_b)
%ENERGIA MEDIA DE SIMBOLO
Es_b = sum(abs(TC_qpsk).^2)/M_b;

%ENERGIA MEDIA DE BIT
Eb_b = (Es_b/k_b)*(n_k);

% Definir un rango de valores para Eb/N0 en dB
%EbN0dB = [0 0.25 0.45 0.65 0.85 1 1.2 1.5 2 2.5 3.5 4 5];
EbN0dB_b = [0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9 10 11 12 13 14];

% Inicializar un vector para almacenar los valores de BER
BER_b = zeros(size(EbN0dB_b));

% Calcular la BER para cada valor de Eb/N0
for i_b = 1:length(EbN0dB_b)
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_b = 10^(EbN0dB_b(i_b)/10);
    
    % Calcular N0
    N0_b = (Eb_b/EbN0_b)*n_k;   %%%%ASIII???

    %GENERAR VECTOR DE RUIDO
    n_b = randn(size(simbM_b))*sqrt(N0_b/2);


    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
    simbR_b = simbM_b + n_b;


    % Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación BPSK
    dis_euclide_b = abs(repmat(simbR_b,M_b,1) - repmat(TC_qpsk.',size(simbR_b)));
    %DISTANCIA A 1 y -1

    % Encontrar el índice del símbolo más cercano para cada valor recibido
    [~, ind1_b] = min(dis_euclide_b);

    % Demodular los valores recibidos
    simbR_demodulado_b = (ind1_b - 1);

    % Convertir los símbolos demodulados a una secuencia de bits
    bits_demodulados_b = int2bit(simbR_demodulado_b,k_b);

    bits_demodulados_reshape = reshape(bits_demodulados_b, R, []);

    % Sumar los valores de cada columna de bits_demodulados_reshape
    suma_bits = sum(bits_demodulados_reshape);

    % Crear matriz vacía
    matriz_bits = [];

    % Recorrer vector de sumas
    for i = 1:length(suma_bits)
       if suma_bits(i) < 3/2
          % Si la suma es menor que 3/2, agregar un 0 a la matriz
          matriz_bits = [matriz_bits; 0];
      else
          % Si la suma es mayor o igual que 3/2, agregar un 1 a la matriz
          matriz_bits = [matriz_bits; 1];
      end
    end

    matriz_bits = reshape(matriz_bits, 1, []);

    % Comparar los bits demodulados con los originales
    errores_b = sum(bits_b ~= matriz_bits);
    
    BER_b(i_b) = errores_b/N_b;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Reshape el vector de bits a una matriz k x N/k
M_q = length(TC_qpsk); %simbolos de modulacion
k_q = log2(M_q);   % Número de bits por símbolo
N_q = length(bits_b); % Número total de bits

bits_reshape_q = reshape(bits_b, k_q, []);


ind_q = bit2int(bits_reshape_q,k_q); % convierte cada columna de X en un entero

simbM_q=TC_qpsk(ind_q+1);

%ENERGIA MEDIA DE SIMBOLO
Es_q = sum(abs(TC_qpsk).^2)/M_q;

%ENERGIA MEDIA DE BIT
Eb_q = Es_q/k_q;

% Calcular la BER para cada valor de Eb/N0
for i_q = 1:length(EbN0dB_b)
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_q = 10^(EbN0dB_b(i_q)/10);

    % Calcular N0
    N0_q = Eb_q/EbN0_q;

    %GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
    n_q = (randn(size(simbM_q)) + 1j*randn(size(simbM_q)))*sqrt(N0_q/2);

    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL AWGN 
    simbR_q=simbM_q+n_q;

    %DISTANCIA EUCLIDEA ENTRE LOS VALORES RECIBIDOS Y LOS SIMBOLOS EN LA CONSTELACION QPSK 
    dis_euclide_q=abs(repmat(simbR_q,M_q,1)-repmat(TC_qpsk.',1,length(simbR_q)));
    
    [~,pos_min_q]=min(dis_euclide_q);
    
     %CONVERTIR SIMBOLOS A BITS
     bits_demodulados_q=int2bit(pos_min_q-1,k_q);
     bits_demodulados_q=reshape(bits_demodulados_q,[],N_q);
     
     errores_q = sum(bits_b ~= bits_demodulados_q);
    
     BER_q(i_q) = errores_q/N_q;
     

end
media_q = mean(BER_q);
media_b = mean(BER_b);

GANANCIA = media_q - media_b;





% Dibujar las curvas BER frente a Eb/N0 para BPSK y QPSK
figure;
semilogy(EbN0dB_b, BER_b,'b-','LineWidth',2);
hold on;
semilogy(EbN0dB_b, BER_q,'g-','LineWidth',2);
hold on;
xlabel('Eb/N0 (dB)');
ylabel('BER');
title('Curva BER QPSK');
legend('QPSK CODIFICACION','QPSK');


