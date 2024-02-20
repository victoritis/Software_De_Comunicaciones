clear all;
%MODULACION QPSK COD

%QPSK Gray Mapping
Ep=2; 
TC_qpsk=[1 1*1j -1*1j -1];


G = [1,1,1,0,0,0,0;1,0,0,1,1,0,0;0,1,0,1,0,1,0;1,1,0,1,0,0,1];

H = [1,0,1,0,1,0,1;0,1,1,0,0,1,1;0,0,0,1,1,1,1];

H_t = H';


%GENERACION DE BITS
% Generar un vector de bits aleatorios de longitud 6
N = 1600;
bits = randi([0, 1], 1, N);
disp(bits);

% Reshape el vector de bits a una matriz k x N/k
M_b = length(TC_qpsk); %simbolos de modulacion
k_b = log2(M_b);   % Número de bits por símbolo
N_b = length(bits); % Número total de bits
n_k = 7/4; % n bits transmitidos/bits informacion


%CODIFICAR

% Inicializar la matriz de bits codificados
bits_codificados = [];

% Dividir los bits en grupos de cuatro y codificar cada grupo
for i = 1:4:length(bits)
    % Seleccionar un grupo de 4 bits
    grupo_bits = bits(i:i+3);

    % Codificar el grupo de bits
    bits_codificados_grupo = mod(grupo_bits * G, 2);

    % Concatenar los bits codificados al final de la matriz de bits codificados
    bits_codificados = [bits_codificados bits_codificados_grupo];
end



% Mostrar la secuencia de bits codificados resultante
disp(bits_codificados);

br = bits_codificados.';

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
EbN0dB = [0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9 10 11 12 13 14];


% Inicializar un vector para almacenar los valores de BER
BER = zeros(size(EbN0dB));

%%%%%%%%%

% Calcular la BER para cada valor de Eb/N0
for j = 1:length(EbN0dB)
   
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_b = 10^(EbN0dB(j)/10);
    
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

        
    % Dividir los bits recibidos en grupos de 7 y decodificar cada grupo
    bits_decodificados = [];
     bits_decodificados = [];
    for i = 1:7:length(bits_demodulados_b)
        % Seleccionar un grupo de 7 bits
        grupo_bits = bits_demodulados_b(i:i+6);
        GP=grupo_bits;
        % Multiplicar el grupo de bits por H'
        bits_codificados_grupo = mod(grupo_bits * H_t, 2);
    
        % Dar la vuelta a los bits
        bits_codificados_grupo = fliplr(bits_codificados_grupo);
    
        bits_codificados_grupo = reshape(bits_codificados_grupo, [], 1);
        
       
        
        SC = [0;0;0]; %C de solucion correcta
        % Verificar si los bits decodificados son iguales a la secuencia 000
        if isequal(bits_codificados_grupo, SC)
            % Si es así, se transmite correctamente, por lo que nos quedamos
            % con los bits de información en las posiciones 3,5,6 y 7 del
            % conjunto de 7 bits original
            bits_decodificados = [bits_decodificados grupo_bits([3 5 6 7])];
        else
            % Comparar cada columna de H con bits_codificados_grupo
            found = false;
            count = 0;%saber en que posicion esta comparando la columna para luego cambiar
            for col = 1:size(H, 2)
                count = count + 1;
                if isequal(H(:,col), bits_codificados_grupo)
                   % Encontrar la posición correspondiente en grupo_bits y cambiar el bit
                    %pos = (grupo-1)*7 + 3 + col;
                    if(GP(count)==0)
                        GP(count)=1;
                    else
                        GP(count)=0;
                    end
                    grupo_bits = GP; %Nose si aqui cambia correctamente el bit de esa posicion
                    bits_decodificados = [bits_decodificados grupo_bits([3 5 6 7])];
                    found = true;
                    break;
                end
            end
    
            % Si no se encontró una secuencia válida, utilizar las posiciones 3, 5, 6 y 7
            if ~found
                bits_decodificados = [bits_decodificados grupo_bits([3 5 6 7])];
            end
    
    
        end
    end
    errores = sum(bits ~= bits_decodificados);
        
    BER(j) = errores/N_b;

end 






%QPSK Gray Mapping
M = 4;
Ep=2; phi=(2*pi/M)*(0:M-1);
TC_qpsk=[1 1*1j -1*1j -1];

%16-QAM Gray Mapping
M=16;
x=0:M-1;
TC_16qam=qammod(x,M,'gray');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%PARTE QPSK

% Reshape el vector de bits a una matriz k x N/k
M_q = length(TC_qpsk); %simbolos de modulacion
k_q = log2(M_q);   % Número de bits por símbolo
N_q = length(bits); % Número total de bits
n_k = 7/4; % n bits transmitidos/bits informacion


bits_reshape_q = reshape(bits, k_q, []);


ind_q = bit2int(bits_reshape_q,k_q); % convierte cada columna de X en un entero

simbM_q=TC_qpsk(ind_q+1);

%ENERGIA MEDIA DE SIMBOLO
Es_q = sum(abs(TC_qpsk).^2)/M_q;

%ENERGIA MEDIA DE BIT
Eb_q = Es_q/k_q;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    %%QPSK
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_q = 10^(EbN0dB(i)/10);

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
     
     errores_q = sum(bits ~= bits_demodulados_q);
    
     BER_q(i) = errores_q/N_q;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%16-QAM
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_16qam = 10^(EbN0dB(i)/10);

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
    bits_demodulados_16qam=reshape(bits_demodulados_16qam,[],N);

    % Comparar los bits demodulados con los originales
    errores_16qam = sum(bits ~= bits_demodulados_16qam);

    % Calcular la tasa de error de bits (BER)
    BER_16qam(i) = errores_16qam/N;
    %disp(mat2str(BER_16qam));

end 





%%16-QAM codificacion
% Reshape el vector de bits a una matriz k x N/k
M_16c = length(TC_16qam); %simbolos de modulacion
k_16c = log2(M_16c);   % Número de bits por símbolo
N_16c = length(bits); % Número total de bits
n_16c = 7/4; % n bits transmitidos/bits informacion


%CODIFICAR

% Inicializar la matriz de bits codificados
bits_codificados_16c = [];

% Dividir los bits en grupos de cuatro y codificar cada grupo
for i = 1:4:length(bits)
    % Seleccionar un grupo de 4 bits
    grupo_bits_16c = bits(i:i+3);

    % Codificar el grupo de bits
    bits_codificados_grupo_16c = mod(grupo_bits_16c * G, 2);

    % Concatenar los bits codificados al final de la matriz de bits codificados
    bits_codificados_16c = [bits_codificados_16c bits_codificados_grupo_16c];
end


br_16c = bits_codificados_16c.';

%bits_reshape = reshape(bits, k, N/k);
bits_reshape_16c = reshape(bits_codificados_16c, k_16c, []);
disp(bits_reshape_b)
ind_16c = bit2int(bits_reshape_16c,k_16c); % convierte cada columna de X en un entero
simbM_16c=TC_16qam(ind_16c+1);

%ENERGIA MEDIA DE SIMBOLO
Es_16c = sum(abs(TC_16qam).^2)/M_16c;

%ENERGIA MEDIA DE BIT
Eb_16c = (Es_16c/k_16c)*(n_k);

% Definir un rango de valores para Eb/N0 en dB
EbN0dB_16c = [0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9 10 11 12 13 14];


% Inicializar un vector para almacenar los valores de BER
BER_16c = zeros(size(EbN0dB));

%%%%%%%%%

% Calcular la BER para cada valor de Eb/N0
for j = 1:length(EbN0dB_16c)
   
      %%16-QAM
    % Calcular el valor de Eb/N0 correspondiente
    EbN0_16c = 10^(EbN0dB(j)/10);

    % Calcular N0
    N0_16c = Eb_16c/EbN0_16c;

    %GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
    n_16c = (randn(size(simbM_16c)) + 1j*randn(size(simbM_16c)))*sqrt(N0_16c/2);

    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
    simbR_16c = simbM_16c + n_16c;

     % Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación 16-QAM
    dis_euclide_16c = abs(repmat(simbR_16c,M_16c,1) - repmat(TC_16qam.',size(simbR_16c)));


    % Encontrar el símbolo de la constelación más cercano para cada valor recibido
    [~, pos_min_16c] = min(dis_euclide_16c,[],1);

    simbR_demodulado_16c=TC_16qam(pos_min_16c);

    %CONVERTIR SIMBOLOS DEMODULADOS A BITS
    bits_demodulados_16c=int2bit(pos_min_16c-ones(size(pos_min_16c)),k_16c);
    bits_demodulados_16c=reshape(bits_demodulados_16c,1,[]);
    
    
    % Dividir los bits recibidos en grupos de 7 y decodificar cada grupo
    bits_decodificados_16c = [];
    for i = 1:7:length(bits_demodulados_16c)
        % Seleccionar un grupo de 7 bits
        grupo_bits_16c = bits_demodulados_16c(i:i+6);
        GP_16c=grupo_bits_16c;
        % Multiplicar el grupo de bits por H'
        bits_codificados_grupo_16c = mod(grupo_bits_16c * H_t, 2);
    
        % Dar la vuelta a los bits
        bits_codificados_grupo_16c = fliplr(bits_codificados_grupo_16c);
    
        bits_codificados_grupo_16c = reshape(bits_codificados_grupo_16c, [], 1);
        
       
        
        SC_16c = [0;0;0]; %C de solucion correcta
        % Verificar si los bits decodificados son iguales a la secuencia 000
        if isequal(bits_codificados_grupo_16c, SC_16c)
            % Si es así, se transmite correctamente, por lo que nos quedamos
            % con los bits de información en las posiciones 3,5,6 y 7 del
            % conjunto de 7 bits original
            bits_decodificados_16c = [bits_decodificados_16c grupo_bits_16c([3 5 6 7])];
        else
            % Comparar cada columna de H con bits_codificados_grupo
            found_16c = false;
            count_16c = 0;%saber en que posicion esta comparando la columna para luego cambiar
            for col_16c = 1:size(H, 2)
                count_16c = count_16c + 1;
                if isequal(H(:,col_16c), bits_codificados_grupo_16c)
                   % Encontrar la posición correspondiente en grupo_bits y cambiar el bit
                    %pos = (grupo-1)*7 + 3 + col;
                    if(GP_16c(count_16c)==0)
                        GP_16c(count_16c)=1;
                    else
                        GP_16c(count_16c)=0;
                    end
                    grupo_bits_16c = GP_16c; %Nose si aqui cambia correctamente el bit de esa posicion
                    bits_decodificados_16c = [bits_decodificados_16c grupo_bits_16c([3 5 6 7])];
                    found_16c = true;
                    break;
                end
            end
    
            % Si no se encontró una secuencia válida, utilizar las posiciones 3, 5, 6 y 7
            if ~found_16c
                bits_decodificados_16c = [bits_decodificados_16c grupo_bits_16c([3 5 6 7])];
            end
    
    
        end
    end
    errores_16c = sum(bits ~= bits_decodificados_16c);
        
    BER_16c(j) = errores_16c/N_16c;

end 













% Dibujar las curvas BER frente a Eb/N0 para BPSK y QPSK
figure;
semilogy(EbN0dB, BER,'b-','LineWidth',2);
hold on;
semilogy(EbN0dB, BER_q,'g-','LineWidth',2);
hold on;
semilogy(EbN0dB, BER_16qam,'c-','LineWidth',2); %%AZUL
hold on;
semilogy(EbN0dB, BER_16c,'r-','LineWidth',2); 
hold on;
xlabel('Eb/N0 (dB)');
ylabel('BER');
title('Curva BER para,QPSK,16-QAM');
legend('QPSK COD','QPSK','16-QAM','16-QAM COD');

