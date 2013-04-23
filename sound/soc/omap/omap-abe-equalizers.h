/*
 * additional ABE equalizers
 */

struct abe_eq_profile {
	char *name;
	unsigned int length;
	s32 coeffs[NBEQ1];
};

struct abe_equalizer {
	char *name;
	unsigned int count;
	struct abe_eq_profile *profiles;
};


/* Matlab programm to generate equalizer coefficients:

function []=DL2_EQUALIZER()
  Fs = 48000; % Sampling Frequency

  N = 3;    % Order
  Fc = 800; % Cutoff frequency
  [b, a] = butter(N, Fc/(Fs/2), 'high');

  fprintf(1, 'NUM'); print_ABE_data2(b, length(b), 0);
  fprintf(1, 'DEN'); print_ABE_data2((-1) * a, length(a), 1);
endfunction

function []=MIC_EQUALIZER()
  Fs = 96000; % Sampling Frequency

  N1 = 8;        % Order
  Fpass = 19800; % Passband frequency
  Apass = 0.1;   % Passband Ripple (dB)
  Astop = 90;    % Stopband Attenuation (dB)
  [b1, a1] = ellip(N1, Apass, Astop, Fpass/(Fs/2));

  % DC removal
  N2 = 1;  % Order
  Fc = 50; % Cutoff frequency
  [b2, a2] = butter(N2, Fc/(Fs/2), 'high');

  b = conv(b1, b2);
  a = conv(a1, a2);

  fprintf(1, 'NUM'); print_ABE_data2(b, length(b), 0);
  fprintf(1, 'DEN'); print_ABE_data2((-1) * a, length(a), 1);
endfunction

%  Quantize coefficients
function [res]=print_ABE_data2(a_, n, k)
  a = floor(a_ * power(2, 28));
  b = a;
  for i=1:length(a_)
    if ((a(i) >= -2^21) && (a(i) < 2^21))
      b(i) = round(a(i)) * 4 * power(2, -6);
      a(i) = round(a(i)) * 4 + 3;
    elseif ((a(i) >= -2^27) && (a(i) <= (2^27 - 2^6)))
      b(i) = round(a(i) / (2^6)) * 4;
      a(i) = round(a(i) / (2^6)) * 4 ;
    elseif ((a(i) >= -2^28) && (a(i) <= (2^28 - 2^7)))
      b(i) = round(a(i) / (2^7)) * 4 * power(2, 1);
      a(i) = round(a(i) / (2^7)) * 4 + 1;
    else
      b(i) = round(a(i) / (2^12));
      a(i) = round(a(i) / (2^12));
      if (a(i) >= 2^21)
        b(i) = 2^21 - 1;
        a(i) = 2^21 - 1;
        disp('coefficient overflow');
      elseif (a(i) < -2^21)
        b(i) = -2^21;
        a(i) = -2^21;
        disp('coefficient underflow');
      end
      b(i) = round(a(i)) * 4 * power (2, 6);
      a(i) = round(a(i)) * 4 + 2;
    end
  end
  for i=n:-1:1+k
    fprintf(1, ' %d,', a(i));
  end
  fprintf(1, '\n');
  res = b / power(2, 24);
endfunction

function []=plot_filter(Fs, a, b)
  F = Fs / 1000;
  nfft = 80000;

  [h, w] = freqz(b, a, nfft);
  h(1)= []; w(1)= [];

  clf;
  xlabel('Frequency kHz');
  grid on; hold on;
  plot(0.5*F*w/pi, 20*log10(abs(h)), 'b');
endfunction

 *
 * The coefficents are loaded using the following format:
 *	s32 DL2_COEF[25] = {
 *		b12, b11, b10, b9, b8, b7, b6, b5, b4, b3, b2, b1, b0,	// NUM 
 *		a12, a11, a10, a9, a8, a7, a6, a5, a4, a3, a2, a1	// DEN
 *	};
 *	s32 MIC_COEF[25] = {
 *		b9, b8, b7, b6, b5, b4, b3, b2, b1, b0,	// NUM
 *		a9, a8, a7, a6, a5, a4, a3, a2, a1,	// DEN
 *		0, 0, 0, 0, 0, 0
 *	};
 */

#define DL2_EQ_COEFFS 25
#define MIC_EQ_COEFFS 19

static struct abe_eq_profile adb_eq_dl2[] =
{
	{
		.name = "Band-pass 450-14500Hz",
		.length = DL2_EQ_COEFFS,
		.coeffs = {
			0, 0, 0, 0, 0, 0, -4063564, 3, 6095349, 3, -6095347, 3, 4063564,
			0, 0, 0, 0, 0, 0, 791336, 3959220, -4483343, 8306648, -407226, 605350,
		}
	},
	{
		.name = "High-pass 600Hz",
		.length = DL2_EQ_COEFFS,
		.coeffs = {
			0, 0, 0, 0, 0, 0, -5916684, 3, 277346, 3, -277342, 3, 5916684,
			0, 0, 0, 0, 0, 0, 2033132, 3902624, -4398119, -7527112, -551244, 432686,
		}
	},
};

static const struct abe_equalizer abe_add_eq[] = {
	{
		.name = "DL2 Left Equalizer",
		.count = ARRAY_SIZE(adb_eq_dl2),
		.profiles = adb_eq_dl2
	},
	{
		.name = "DL2 Right Equalizer",
		.count = ARRAY_SIZE(adb_eq_dl2),
		.profiles = adb_eq_dl2
	},
};
