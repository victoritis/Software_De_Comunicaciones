


%BPSK (Binary Phase Shift Keying)

%En el caso de la BPSK, la parte imaginaria es siempre cero, por lo que los números complejos son solo 1 y -1.
%Estos números se pueden representar como puntos en el eje real del plano complejo.
TC_b = [-1 1];

% Generar un vector de bits aleatorios de longitud 6
bits_b = randi([0, 1], 1, 1000);

% Reshape el vector de bits a una matriz k x N/k
M_b = length(TC_b); %simbolos de modulacion
k_b = log2(M_b);   % Número de bits por símbolo
N_b = length(bits_b); % Número total de bits
%bits_reshape = reshape(bits, k, N/k);
bits_reshape_b = reshape(bits_b, k_b, []);



ind_b = bit2int(bits_reshape_b,k_b); % convierte cada columna de X en un entero
simbM_b=TC_b(ind_b+1);

%ENERGIA MEDIA DE SIMBOLO
Es_b = sum(abs(TC_b).^2)/M_b;


%ENERGIA MEDIA DE BIT
Eb_b = Es_b/k_b;


% Definir el valor de Eb/N0 en decibelios
x_b = 0; % 10 dB

% Calcular el valor de Eb/N0 correspondiente
EbN0_b = 10^(x_b/10);

% Calcular N0
N0_b = Eb_b/EbN0_b;

%GENERAR VECTOR DE RUIDO
n_b = randn(size(simbM_b))*sqrt(N0_b/2);


%VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
simbR_b = simbM_b + n_b;


% Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación BPSK
dis_euclide_b = abs(repmat(simbR_b,M_b,1) - repmat(TC_b.',size(simbR_b)));
%DISTANCIA A 1 y -1

% Encontrar el índice del símbolo más cercano para cada valor recibido
[~, ind1_b] = min(dis_euclide_b);

% Demodular los valores recibidos
simbR_demodulado_b = (ind1_b - 1);

% Convertir los símbolos demodulados a una secuencia de bits
bits_demodulados_b = int2bit(simbR_demodulado_b,k_b);


% Comparar los bits demodulados con los originales
errores_b = sum(bits_b ~= bits_demodulados_b);

% Calcular la tasa de error de bits (BER)
BER_b = errores_b/N_b;

%APARTADO GRAFICA DE VALORES BER

% Definir un rango de valores para Eb/N0 en dB
%EbN0dB = [0 0.25 0.45 0.65 0.85 1 1.2 1.5 2 2.5 3.5 4 5];
EbN0dB_b = [-100,-97,-94,-91,-88,-85,-82,-79,-76,-73,-70,-67,-64 ,-61 ,-58 ,-55 ,-52 ,-49 ,-46 ,-43 ,-40 ,-37 ,-34 ,-31 , -28 , -25 , -22 , -19 , -16 , -13];

% Inicializar un vector para almacenar los valores de BER
BER_b = zeros(size(EbN0dB_b));

% Calcular la BER para cada valor de Eb/N0
for i_b = 1:length(EbN0dB_b)
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_b = 10^(EbN0dB_b(i_b)/10);

    % Calcular N0
    N0_b = Eb_b/EbN0_b;

    %GENERAR VECTOR DE RUIDO
    n_b = randn(size(simbM_b))*sqrt(N0_b/2);

    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
    simbR_b = simbM_b + n_b;

    % Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación BPSK
    dis_euclide_b = abs(repmat(simbR_b,M_b,1) - repmat(TC_b.',size(simbR_b)));

    % Encontrar el índice del símbolo más cercano para cada valor recibido
    [~, ind1_b] = min(dis_euclide_b);

    % Demodular los valores recibidos
    simbR_demodulado_b = (ind1_b - 1);

    % Convertir los símbolos demodulados a una secuencia de bits
    bits_demodulados_b = int2bit(simbR_demodulado_b,k_b);

    % Comparar los bits demodulados con los originales y calcular la tasa de error de bits (BER)
    errores_b = sum(bits_b ~= bits_demodulados_b);
    
     BER_b(i_b) = errores_b/N_b;
end


%BPSK YA ESTA EN GRAY MAPPING DE POR SI SOLO





%QPSK (Quadrature Phase Shift Keying)

% En el caso de la QPSK, los números complejos son 1 + 1i, -1 + 1i, -1 - 1i y 1 - 1i.
% Estos números se pueden representar como puntos en el plano complejo.
TC_q = [1 ,1j,  -1, -1j];


% Generar un vector de bits aleatorios de longitud N
N_q = 1000;
bits_q = randi([0, 1], 1, N_q);

% Reshape el vector de bits a una matriz k x N/k
M_q = length(TC_q); %simbolos de modulacion
k_q = log2(M_q);   % Número de bits por símbolo
N_q = length(bits_q); % Número total de bits

bits_reshape_q = reshape(bits_q, k_q, []);


ind_q = bit2int(bits_reshape_q,k_q); % convierte cada columna de X en un entero

simbM_q=TC_q(ind_q+1);

%ENERGIA MEDIA DE SIMBOLO
Es_q = sum(abs(TC_q).^2)/M_q;

%ENERGIA MEDIA DE BIT
Eb_q = Es_q/k_q;

% Definir el valor de Eb/N0 en decibelios
x_q = 0; % dB

% Calcular el valor de Eb/N0 correspondiente
EbN0_q = 10^(x_q/10);

% Calcular N0
N0_q = Eb_q/EbN0_q;

%GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
n_q = (randn(size(simbM_q)) + 1j*randn(size(simbM_q)))*sqrt(N0_q/2);

%VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL AWGN 
simbR_q=simbM_q+n_q;


%DISTANCIA EUCLIDEA ENTRE LOS VALORES RECIBIDOS Y LOS SIMBOLOS EN LA CONSTELACION QPSK 
dis_euclide_q=abs(repmat(simbR_q,M_q,1)-repmat(TC_q.',1,length(simbR_q)));
[val_min_q,pos_min_q]=min(dis_euclide_q);
simbR_demodulado_q=TC_q(pos_min_q);

%CONVERTIR SIMBOLOS A BITS
bits_demodulados_q=int2bit(pos_min_q-1,k_q);
bits_demodulados_q=reshape(bits_demodulados_q,[],N_q);

% Comparar los bits demodulados con los originales
errores_q = sum(bits_q ~= bits_demodulados_q);

% Calcular la tasa de error de bits (BER)
BER_q = errores_q/N_q;

% Definir un rango de valores para Eb/N0 en dB
EbN0dB_q = [-100,-97,-94,-91,-88,-85,-82,-79,-76,-73,-70,-67,-64 ,-61 ,-58 ,-55 ,-52 ,-49 ,-46 ,-43 ,-40 ,-37 ,-34 ,-31 , -28 , -25 , -22 , -19 , -16 , -13];

% Inicializar un vector para almacenar los valores de BER
BER_q = zeros(size(EbN0dB_q));

% Calcular la BER para cada valor de Eb/N0
for i_q = 1:length(EbN0dB_q)
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_q = 10^(EbN0dB_q(i_q)/10);

    % Calcular N0
    N0_q = Eb_q/EbN0_q;

    %GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
    n_q = (randn(size(simbM_q)) + 1j*randn(size(simbM_q)))*sqrt(N0_q/2);

    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL AWGN 
    simbR_q=simbM_q+n_q;

    %DISTANCIA EUCLIDEA ENTRE LOS VALORES RECIBIDOS Y LOS SIMBOLOS EN LA CONSTELACION QPSK 
    dis_euclide_q=abs(repmat(simbR_q,M_q,1)-repmat(TC_q.',1,length(simbR_q)));
    
    [~,pos_min_q]=min(dis_euclide_q);
    
     %CONVERTIR SIMBOLOS A BITS
     bits_demodulados_q=int2bit(pos_min_q-1,k_q);
     bits_demodulados_q=reshape(bits_demodulados_q,[],N_q);
     
     errores_q = sum(bits_q ~= bits_demodulados_q);
    
     BER_q(i_q) = errores_q/N_q;
end


%CON GRAY MAPPING QUEDARA ASI : TC_q = [1+1j ,  1-1j , -  1+  1j , -  1-   1 j];



% 16-QAM (Quadrature Amplitude Modulation)

% En el caso de la 16-QAM, los números complejos son -3 -3i, -3 -1i, -3 +1i, -3 +3i, -1 -3i, -1 -1i, -1 +1i,
TC_16qam =[-3-3j, -3-1j, -3+3j, -3+1j, -1-3j, -1-1j, -1+3j, -1+1j, 3-3j, 3-1j, 3+3j , 3+1j , 1-3j , 1-1j , 1+3j , 1+1j];

% Generar un vector de bits aleatorios de longitud N
N_16qam = 1600;
bits_16qam = randi([0, 1], 1, N_16qam);

% Reshape el vector de bits a una matriz k x N/k
M_16qam = length(TC_16qam); %simbolos de modulacion
k_16qam = log2(M_16qam);   % Número de bits por símbolo


bits_reshape_16qam = reshape(bits_16qam', k_16qam,[]);

ind_16qam = bit2int(bits_reshape_16qam,k_16qam); % convierte cada columna de X en un entero

simbM_16qam=TC_16qam(ind_16qam+1);

%ENERGIA MEDIA DE SIMBOLO
Es_16qam = sum(abs(TC_16qam).^2)/M_16qam;

%ENERGIA MEDIA DE BIT
Eb_16qam = Es_16qam/k_16qam;

% Definir el valor de Eb/N0 en decibelios
x_16qam = 0; % dB

% Calcular el valor de Eb/N0 correspondiente
EbN0_16qam = 10^(x_16qam/10);

% Calcular N0
N0_16qam = Eb_16qam/EbN0_16qam;

%GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
n_16qam = (randn(size(simbM_16qam)) + 1j*randn(size(simbM_16qam)))*sqrt(N0_16qam/2);


%VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL AWGN 
simbR_16qam=simbM_16qam+n_16qam;


%DISTANCIA EUCLIDEA ENTRE LOS VALORES RECIBIDOS Y LOS SIMBOLOS EN LA CONSTELACION QPSK 
dis_euclide_16qam=abs(repmat(simbR_16qam,M_16qam,1)-repmat(TC_16qam.',1,length(simbR_16qam)));
[val_min,pos_min_16qam]=min(dis_euclide_16qam,[],1);
simbR_demodulado_16qam=TC_16qam(pos_min_16qam);

%CONVERTIR SIMBOLOS DEMODULADOS A BITS
bits_demodulados_16qam=int2bit(pos_min_16qam-1,k_16qam);
bits_demodulados_16qam=reshape(bits_demodulados_16qam,[],N_16qam);

% Comparar los bits demodulados con los originales
errores_16qam = sum(bits_16qam ~= bits_demodulados_16qam);

% Calcular la tasa de error de bits (BER)
BER_16qam = errores_16qam/N_16qam;
disp(mat2str(BER_16qam));

%APARTADO GRAFICA DE VALORES BER

% Definir un rango de valores para Eb/N0 en dB
EbN0dB_16qam = [-100,-97,-94,-91,-88,-85,-82,-79,-76,-73,-70,-67,-64 ,-61 ,-58 ,-55 ,-52 ,-49 ,-46 ,-43 ,-40 ,-37 ,-34 ,-31 , -28 , -25 , -22 , -19 , -16 , -13];

% Inicializar un vector para almacenar los valores de BER
BER_16qam = zeros(size(EbN0dB_16qam));


% Calcular la BER para cada valor de Eb/N0
for i_16qam = 1:length(EbN0dB_16qam)
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_16qam = 10^(EbN0dB_16qam(i_16qam)/10);

    % Calcular N0
    N0_16qam = Eb_16qam/EbN0_16qam;

    %GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
    n_16qam = (randn(size(simbM_16qam)) + 1j*randn(size(simbM_16qam)))*sqrt(N0_16qam/2);

    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
    simbR_16qam = simbM_16qam + n_16qam;

     % Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación 16-QAM
    dis_euclide_16qam = abs(repmat(simbR_16qam,M_16qam,1) - repmat(TC_16qam.',size(simbR_16qam)));


    % Encontrar el símbolo de la constelación más cercano para cada valor recibido
    [~, pos_min_16qam] = min(dis_euclide_16qam,[],1);

    simbR_demodulado_16qam=TC_16qam(pos_min_16qam);

    %CONVERTIR SIMBOLOS DEMODULADOS A BITS
    bits_demodulados_16qam=int2bit(pos_min_16qam-ones(size(pos_min_16qam)),k_16qam);
    bits_demodulados_16qam=reshape(bits_demodulados_16qam,[],N_16qam);

    % Comparar los bits demodulados con los originales
    errores_16qam = sum(bits_16qam ~= bits_demodulados_16qam);

    % Calcular la tasa de error de bits (BER)
    BER_16qam(i_16qam) = errores_16qam/N_16qam;
    %disp(mat2str(BER_16qam));

end 


%GRAY MAPING SERIA : TC [-3+3j,-3+1j,-1+3j,-1+1j,-3-3j,-3-1j,-1-3j,-1-1j,3+3j,3+1j,1+3j,1+1j,3-3j,3-1j,1-3j,1-1j]


% 64-QAM (Quadrature Amplitude Modulation)

% En el caso de la 64-QAM, los números complejos son -7 -7i, -7 -5i,...,-7 +7i,
% -5 -7i,...,-5 +7i,...,+7 +7i
TC = [];
for i = -7:2:7
    for j = -7:2:7
        TC = [TC i+j*1j];
    end
end


% Generar un vector de bits aleatorios de longitud N
N = 4800;
bits = randi([0, 1], 1, N);
disp(['BITS : ' mat2str(bits)]);

M = length(TC); %simbolos de modulacion
k = log2(M);   % Número de bits por símbolo


bits_reshape = reshape(bits', k,[]);

ind = bit2int(bits_reshape,k); % convierte cada columna de X en un entero
simbM=TC(ind+1);

%ENERGIA MEDIA DE SIMBOLO
Es = sum(abs(TC).^2)/M;

%ENERGIA MEDIA DE BIT
Eb = Es/k;

% Definir el valor de Eb/N0 en decibelios
x = 0; % dB

% Calcular el valor de Eb/N0 correspondiente
EbN0 = 10^(x/10);

% Calcular N0
N0 = Eb/EbN0;

%GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
n = (randn(size(simbM)) + 1j*randn(size(simbM)))*sqrt(N0/2);


%VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL AWGN 
simbR=simbM+n;


%DISTANCIA EUCLIDEA ENTRE LOS VALORES RECIBIDOS Y LOS SIMBOLOS EN LA CONSTELACION QPSK 
dis_euclide=abs(repmat(simbR,M,1)-repmat(TC.',1,length(simbR)));
[val_min,pos_min]=min(dis_euclide,[],1);
simbR_demodulado=TC(pos_min);

%CONVERTIR SIMBOLOS DEMODULADOS A BITS
bits_demodulados=int2bit(pos_min-1,k);
bits_demodulados=reshape(bits_demodulados,[],N);

% Comparar los bits demodulados con los originales
errores = sum(bits ~= bits_demodulados);

% Calcular la tasa de error de bits (BER)
BER = errores/N;

%APARTADO GRAFICA DE VALORES BER

% Definir un rango de valores para Eb/N0 en dB
EbN0dB = [-100,-97,-94,-91,-88,-85,-82,-79,-76,-73,-70,-67,-64 ,-61 ,-58 ,-55 ,-52 ,-49 ,-46 ,-43 ,-40 ,-37 ,-34 ,-31 , -28 , -25 , -22 , -19 , -16 , -13];

% Inicializar un vector para almacenar los valores de BER
BER = zeros(size(EbN0dB));


% Calcular la BER para cada valor de Eb/N0
for i = 1:length(EbN0dB)
    % Calcular el valor de Eb/N0 correspondiente
    EbN0 = 10^(EbN0dB(i)/10);

    % Calcular N0
    N0 = Eb/EbN0;

    %GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
    n = (randn(size(simbM)) + 1j*randn(size(simbM)))*sqrt(N0/2);

    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
    simbR = simbM + n;

     % Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación 16-QAM
    dis_euclide = abs(repmat(simbR,M,1) - repmat(TC.',size(simbR)));


    % Encontrar el símbolo de la constelación más cercano para cada valor recibido
    [~, pos_min] = min(dis_euclide,[],1);

    simbR_demodulado=TC(pos_min);

    %CONVERTIR SIMBOLOS DEMODULADOS A BITS
    bits_demodulados=int2bit(pos_min-ones(size(pos_min)),k);
    bits_demodulados=reshape(bits_demodulados,[],N);

    % Comparar los bits demodulados con los originales
    errores = sum(bits ~= bits_demodulados);

    % Calcular la tasa de error de bits (BER)
    BER(i) = errores/N;
    
end  


%PARA GRAY MAPPING SERIA : TC_64qam = [];
%for i = [-7 -5 -1 -3 7 5 1 3]
 %   for j = [-7 -5 -1 -3 7 5 1 3]
  %      TC_64qam = [TC_64qam i+j*1j];
   % end
%end




% Dibujar las curvas BER frente a Eb/N0 para BPSK y QPSK
figure;
semilogy(EbN0dB_b, BER_b,'b-','LineWidth',2);
hold on;
semilogy(EbN0dB_q, BER_q,'g-','LineWidth',2);
hold on;
semilogy(EbN0dB_16qam, BER_16qam,'c-','LineWidth',2);
hold on;
semilogy(EbN0dB, BER,'r-','LineWidth',2);
grid on;
xlabel('Eb/N0 (dB)');
ylabel('BER');
title('Curva BER para BPSK,QPSK,16-QAM,64-QAM');
legend('BPSK','QPSK', '16-QAM', '64-QAM');
