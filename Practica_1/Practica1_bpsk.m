%//QPSK se usan numeros imaginarios
%1  1j  -1 -1j   = s

%BPSK 1 y -1

%QAM   s = qamod(aux.^, M, 'bin')
%aux = 1xM-1


%BPSK (Binary Phase Shift Keying)

%En el caso de la BPSK, la parte imaginaria es siempre cero, por lo que los números complejos son solo 1 y -1.
%Estos números se pueden representar como puntos en el eje real del plano complejo.
TC_b = [-1 1];

% Generar un vector de bits aleatorios de longitud 6
bits_b = randi([0, 1], 1, 100000);

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
EbN0dB_b = [0 1 2 3 4 5 6 7 8 9 10];

%x_b = [10 8 7 6 5 4 3 2 1]; % vector de valores en dB
%EbN0dB_b = arrayfun(@(x) 10^(x/10), x_b); % aplicación de la fórmula a cada valor de x_b


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

% Dibujar la curva BER frente a Eb/N0(dB)
figure;
semilogy(EbN0dB_b,BER_b,'o-','LineWidth',2,'MarkerSize',8,'Color',[1 0.5 0]);
xlabel('Eb/N0(dB)');
ylabel('BER');
title('Curva BER para BPSK');
grid on;


%ESTE YA ES GRAY MAPPING EN SI MISMO






