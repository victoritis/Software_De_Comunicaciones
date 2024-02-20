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
bits_demodulados_16qam=int2bit(pos_min_16qam-ones(size(pos_min_16qam)),k_16qam);
bits_demodulados_16qam=reshape(bits_demodulados_16qam,[],N_16qam);

% Comparar los bits demodulados con los originales
errores_16qam = sum(bits_16qam ~= bits_demodulados_16qam);

% Calcular la tasa de error de bits (BER)
BER_16qam = errores_16qam/N_16qam;
disp(mat2str(BER_16qam));

%APARTADO GRAFICA DE VALORES BER

% Definir un rango de valores para Eb/N0 en dB
EbN0dB_16qam = [-100 -80 -70 -60 -50 -40 -30 -20 -10 0];

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
% Dibujar la curva BER frente a Eb/N
figure;
semilogy(EbN0dB_16qam,BER_16qam,'b-','LineWidth',2);
xlabel('Eb/N0 (dB)');
ylabel('BER');
title('Curva BER para 16-QAM');
grid on;

