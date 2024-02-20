clear all;

%16-QAM Gray Mapping
M=16;
x=0:M-1;
TC_16qam=qammod(x,M,'gray');

%QPSK

%GENERACION DE BITS
% Generar un vector de bits aleatorios de longitud 6
N = 320000;
Bits = randi([0, 1], 1, N);
n_k=3/1;


%CODIFICACION
kc=1;
n=3;
M=3; %Constant lenght: Son el nº de bloques 
array = [7 5 6]; %1º cod convolucional: 7 se corresponderia con c1, 5 con c2
%array = [3 5 6]; %2º cod convolucional: 3 con c1, 5 con c2
trellis = poly2trellis(M, array);
bits = convenc(Bits, trellis); %le paso los bits aleatorios creados y el codificador


%QPSK Gray Mapping
M = 4;
Ep=2; phi=(2*pi/M)*(0:M-1);
TC_qpsk=[1 1*1j -1*1j -1];

M_b = length(TC_qpsk); %simbolos de modulacion
k_b = log2(M_b);   % Número de bits por símbolo
N_b = length(bits); % Número total de bits

bits_reshape_b = reshape(bits, k_b, []);


ind_b = bit2int(bits_reshape_b,k_b); % convierte cada columna de X en un entero

simbM_b=TC_qpsk(ind_b+1);

%ENERGIA MEDIA DE SIMBOLO
Es_b = sum(abs(TC_qpsk).^2)/M_b;

%ENERGIA MEDIA DE BIT
Eb_b = Es_b/k_b;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%PARTE 16-QAM

% Reshape el vector de bits a una matriz k x N/k
M_16qam = length(TC_16qam); %simbolos de modulacion
k_16qam = log2(M_16qam);   % Número de bits por símbolo


bits_reshape_16qam = reshape(bits', k_16qam,[]);

ind_16qam = bit2int(bits_reshape_16qam,k_16qam); % convierte cada columna de X en un entero

simbM_16qam=TC_16qam(ind_16qam+1);

%ENERGIA MEDIA DE SIMBOLO
Es_16qam = sum(abs(TC_16qam).^2)/M_16qam;

%ENERGIA MEDIA DE BIT
Eb_16qam = Es_16qam/k_16qam;

% Definir un rango de valores para Eb/N0 en dB
EbN0dB = [0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9 10 11 12 13 14];

% Inicializar un vector para almacenar los valores de BER
BER_q = zeros(size(EbN0dB));

% Inicializar un vector para almacenar los valores de BER
BER_16qam = zeros(size(EbN0dB));

%%%%%%%%%


% Calcular la BER para cada valor de Eb/N0
for i = 1:length(EbN0dB)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %QPSK
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_b = 10^(EbN0dB(i)/10);
    
    % Calcular N0
    N0_b = (Eb_b/EbN0_b)*n_k;
    
    %GENERAR VECTOR DE RUIDO
    n_b = randn(size(simbM_b))*sqrt(N0_b/2);
    
    
    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
    simbR_b = simbM_b + n_b;
    
    
    % Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación BPSK
    dis_euclide_b = abs(repmat(simbR_b,M_b,1) - repmat(TC_qpsk.',size(simbR_b)));
    %DISTANCIA A 1 y -1
    
    % Encontrar el índice del símbolo más cercano para cada valor recibido
    [~, ind1_b] = min(dis_euclide_b);
    
    %DISTANCIA EUCLIDEA ENTRE LOS VALORES RECIBIDOS Y LOS SIMBOLOS EN LA CONSTELACION QPSK 
    dis_euclide_b=abs(repmat(simbR_b,M_b,1)-repmat(TC_qpsk.',1,length(simbR_b)));
    [val_min_b,pos_min_b]=min(dis_euclide_b);
    simbR_demodulado_b=TC_qpsk(pos_min_b);
    
    %CONVERTIR SIMBOLOS A BITS
     bits_demodulados_b=int2bit(pos_min_b-1,k_b);
     bits_demodulados_b=reshape(bits_demodulados_b,1,[]);
     
     errores_q = sum(bits ~= bits_demodulados_b);
    
     BER_b(i) = errores_q/N_b;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%16-QAM
     %%16-QAM
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_16qam = 10^(EbN0dB(i)/10);

    % Calcular N0
    N0_16qam = Eb_16qam/EbN0_16qam*n_k;

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
    bits_demodulados_16qam=reshape(bits_demodulados_16qam,1,[]);

    bits_decodificados_16qam = vitdec(bits_demodulados_16qam, trellis, M*5, 'trunc', 'hard');

    % Comparar los bits demodulados con los originales
    errores_16qam = sum(Bits ~= bits_decodificados_16qam);

    % Calcular la tasa de error de bits (BER)
    BER_16qam(i) = errores_16qam/N;
    %disp(mat2str(BER_16qam));

end

% Dibujar las curvas BER frente a Eb/N0 para BPSK y QPSK
figure;
semilogy(EbN0dB, BER_b,'b-','LineWidth',2);
hold on;
semilogy(EbN0dB, BER_16qam,'r-','LineWidth',2); 
hold on;
xlabel('Eb/N0 (dB)');
ylabel('BER');
title('Curva BER para,QPSK,16-QAM');
legend('QPSK COD','16-QAM COD');




%Para generar un código convolucional con una ganancia de codificación alta, se debe utilizar un polinomio generador que tenga una buena distancia libre, es decir, una gran separación entre los diferentes caminos que se pueden tomar en el trellis. Una forma de encontrar estos polinomios es utilizando la función "poly2trellis" en Matlab, que permite generar un trellis a partir de un polinomio generador.

% Definir la tasa del codigo y la longitud del registro de desplazamiento
rate = 1/2;
constraint_length = 3;

% Crear objeto de codigo convolucional utilizando poly2trellis
trellis = poly2trellis(constraint_length, [6 7]);
%s(n) = [s1(n), s2(n), s3(n)]

%s1(n) = s2(n-1) + s3(n-1) + d(n)
%s2(n) = s1(n-1) + s3(n-1)
%s3(n) = s2(n-1)

% Calcular la distancia libre del codigo
free_distance = distspec(trellis);

% Generar una secuencia de bits aleatorios para codificar
message = randi([0 1], 1, 100);

% Codificar la secuencia de bits utilizando el objeto de codigo convolucional
codeword = convenc(message, trellis);


