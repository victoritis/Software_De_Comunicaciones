

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
EbN0dB_q = [-100 -80 -70 -60 -50 -40 -30 -20 -10 0 10 20 30];

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

% Dibujar la curva BER frente a Eb/N
figure;
semilogy(EbN0dB_q,BER_q,'s--','LineWidth',2,'MarkerSize',8);
xlabel('Eb/N0 (dB)');
ylabel('BER');
title('Curva BER para QPSK');
grid on;