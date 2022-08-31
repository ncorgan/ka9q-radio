// Read from SoapySDR
// Accept control commands from UDP socket
#define _GNU_SOURCE 1
#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Types.h>

#include <assert.h>
#include <pthread.h>
#include <string.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <locale.h>
#include <sys/time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <syslog.h>
#include <sys/stat.h>
#include <getopt.h>

#include "conf.h"
#include "misc.h"
#include "multicast.h"
#include "decimate.h"
#include "status.h"

// TODO: formatting
// TODO: make sure we close in the error case

// Time in 100 ms update intervals to wait between gain steps
int const HOLDOFF_TIME = 2;

int const Bufsize = 16384;
#define BUFFERSIZE  (1<<21) // Upcalls seem to be 256KB; don't make too big or we may blow out of the cache

// Global variables set by command line options
char *Locale;
int RTP_ttl = 1;
int Status_ttl = 1;
int IP_tos = 48; // AF12 left shifted 2 bits
int Verbose;
int AGC;     // Default to hardware AGC
char *Dev = ""; // Default to first device found
struct SoapySDRDevice *Device; // Set for benefit of closedown()

struct sdrstate {
  struct SoapySDRDevice *device; // Opaque pointer
  struct SoapySDRStream *stream; // Opaque pointer

  char *driver_key;
  char *hardware_key;
  char *hardware_info;

  size_t channel;

  char *frontend_mapping;
  char *antenna;

  double frequency;
  char *frequency_tune_args;

  double gain;
  double sample_rate;
  double bandwidth;

/*
  uint32_t sample_rates[20];
  uint64_t SN; // Serial number
  char *description;
  int samprate; // True sample rate of single A/D converter

  int antenna_bias; // Bias tee on/off

  // Tuning
  double frequency;
  double calibration; // Frequency error
  int frequency_lock;
  char *frequency_file; // Local file to store frequency in case we restart

  // AGC
  int holdoff_counter; // Time delay when we adjust gains
  int linearity; // Use linearity gain tables; default is sensitivity
  int gain;      // Gain passed to manual gain setting

  // Sample statistics
  int clips;  // Sample clips since last reset
  float power;   // Running estimate of A/D signal power
  float DC;      // DC offset for real samples
*/

  int blocksize;// Number of real samples per packet or twice the number of complex samples per packet

  FILE *status;    // Real-time display in /run (currently unused)

  // Multicast I/O
  char *metadata_dest;
  struct sockaddr_storage output_metadata_dest_address;
  uint64_t output_metadata_packets;
  int status_sock;  // Socket handle for outgoing status messages
  int nctl_sock;    // Socket handle for incoming commands (same socket as status)

  uint64_t commands; // Command counter
  uint32_t command_tag; // Last received command tag
  
  char *data_dest;
  struct sockaddr_storage output_data_source_address; // Multicast output socket
  struct sockaddr_storage output_data_dest_address; // Multicast output socket
  int data_sock;     // Socket handle for sending real time stream
  struct rtp_state rtp; // Real time protocol (RTP) state
  int rtp_type;      // RTP type to indicate sample rate, etc (should be rethought)

  pthread_t display_thread;
  pthread_t ncmd_thread;
};

static struct option Options[] =
  {
      /*
    {"iface", required_argument, NULL, 'A'},
    {"pcm-out", required_argument, NULL, 'D'},
    {"iq-out", required_argument, NULL, 'D'},
    {"device", required_argument, NULL, 'I'},
    {"serial", required_argument, NULL, 'I'},
    {"linearity",no_argument, NULL, 'L'},
    {"status-out", required_argument, NULL, 'R'},
    {"ssrc", required_argument, NULL, 'S'},
    {"data-ttl", required_argument, NULL, 'T'},
    {"rtp-ttl", required_argument, NULL, 'T'},
    {"ttl", required_argument, NULL, 'T'}, // Data channel TTL
    {"agc", no_argument, NULL, 'a'},  // Software AGC (default is hardware, preferred)
    {"bias", no_argument, NULL, 'b'}, // Turn on bias tee
    {"calibrate", required_argument, NULL, 'c' },
    {"frequency", required_argument, NULL, 'f'},
    {"tos", required_argument, NULL, 'p'},
    {"iptos", required_argument, NULL, 'p'},
    {"ip-tos", required_argument, NULL, 'p'},    
    {"samprate", required_argument, NULL, 'r'},
    {"samplerate", required_argument, NULL, 'r'},
    {"status-ttl", required_argument, NULL, 't'},
    {"verbose", no_argument, NULL, 'v'},
    {NULL, 0, NULL, 0},
    */
  };
static char const Optstring[] = "A:D:I:LR:S:T:abc:f:p:r:t:v";

//double set_correct_freq(struct sdrstate *sdr,double freq);
void decode_soapysdr_commands(struct sdrstate *,unsigned char *,int);
void send_soapysdr_status(struct sdrstate *,int);
void rx_callback(unsigned char *buf,uint32_t len, void *ctx);
void *display(void *);
void *ncmd(void *);
static void closedown(int a);

int main(int argc,char *argv[]){
  umask(02);
#if 0
  // Dump environment variables
  extern char **environ;
  for(int i=0;;i++){
    if(environ[i])
      printf("environ[%'d]: %s\n",i,environ[i]);
    else
      break;
  }
#endif  

  Locale = getenv("LANG");
  if(Locale == NULL || strlen(Locale) == 0)
    Locale = "en_US.UTF-8";
  setlocale(LC_ALL,Locale);

  setlinebuf(stdout);

  struct sdrstate * const sdr = (struct sdrstate *)calloc(1,sizeof(struct sdrstate));
  /*
  sdr->blocksize = DEFAULT_BLOCKSIZE;
  sdr->samprate = DEFAULT_SAMPRATE;
  */

  int c;
  int ret;
  double init_frequency = NAN;
  double init_gain = NAN;
  double init_sample_rate = NAN;
  double init_bandwidth = NAN;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
      /*
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'D':
      sdr->data_dest = optarg;
      break;
    case 'I':
      Dev = strtoll(optarg,NULL,0);
      break;
    case 'L':
      sdr->linearity = 1;
      break;
    case 'R':
      sdr->metadata_dest = optarg;
      break;
    case 'S':
      sdr->rtp.ssrc = strtol(optarg,NULL,0);
      break;
    case 'T':
      RTP_ttl = strtol(optarg,NULL,0);
      break;
    case 'a':
      AGC = 1;
      break;
    case 'b':
      sdr->antenna_bias++;
      break;
    case 'c':
      sdr->calibration = strtod(optarg,NULL);
      break;
    case 'f':
      init_frequency = parse_frequency(optarg);
      sdr->frequency_lock = 1;
      break;
    case 'p':
      IP_tos = strtol(optarg,NULL,0);
      break;
    case 'r':
      sdr->samprate = strtol(optarg,NULL,0);
      break;
    case 't':
      Status_ttl = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    default:
    case '?':
      fprintf(stderr,"Unknown argument %c\n",c);
      break;
    }
    */
  }
    // Enumerate devices, take first that match args
    size_t device_count = 0;
    SoapySDRKwargs *devices = SoapySDRDevice_enumerateStrArgs(Dev, &device_count);
    if(device_count == 0){
        fprintf(stderr, "No SoapySDR devices found for args: %s\n", Dev);
        exit(1);
    }

    for(size_t i = 0; i < device_count; ++i){
        char *device_str = SoapySDRKwargs_toString(&devices[i]);
        if(device_str){
            fprintf(stderr, "SoapySDR device: %s\n", device_str);
            free(device_str);
        }
    }
    SoapySDRKwargsList_clear(devices, device_count);

    // Open
    sdr->device = SoapySDRDevice_makeStrArgs(Dev);
    if(!sdr->device){
        fprintf(stderr, "Failed to open SoapySDR device with args: %s\n", Dev);
        exit(1);
    }

    // Get device info
    sdr->driver_key = SoapySDRDevice_getDriverKey(sdr->device);
    sdr->hardware_key = SoapySDRDevice_getHardwareKey(sdr->device);

    SoapySDRKwargs hardware_info = SoapySDRDevice_getHardwareInfo(sdr->device);
    sdr->hardware_info = SoapySDRKwargs_toString(&hardware_info);
    SoapySDRKwargs_clear(&hardware_info);

    // Set frontend mapping if specified
    if(sdr->frontend_mapping){
        ret = SoapySDRDevice_setFrontendMapping(sdr->device, SOAPY_SDR_RX, sdr->frontend_mapping);
        if(ret){
            fprintf(stderr, "SoapySDRDevice_setFrontendMapping returned error %d (%s)\n", ret, SoapySDRDevice_lastError());
            exit(1);
        }
    }
    sdr->frontend_mapping = SoapySDRDevice_getFrontendMapping(sdr->device, SOAPY_SDR_RX);

    // Set antenna if specified
    if(sdr->antenna){
        ret = SoapySDRDevice_setAntenna(sdr->device, SOAPY_SDR_RX, sdr->channel, sdr->antenna);
        if(ret){
            fprintf(stderr, "SoapySDRDevice_setAntenna returned error %d (%s)\n", ret, SoapySDRDevice_lastError());
            exit(1);
        }
    }
    sdr->antenna = SoapySDRDevice_getAntenna(sdr->device, SOAPY_SDR_RX, sdr->channel);

    // Set frequency if specified
    if(!isnan(init_frequency)){
        SoapySDRKwargs tune_args;
        SoapySDRKwargs *tune_args_param = NULL;

        if(sdr->frequency_tune_args){
            tune_args = SoapySDRKwargs_fromString(sdr->frequency_tune_args);
            tune_args_param = &tune_args;
        }

        ret = SoapySDRDevice_setFrequency(sdr->device, SOAPY_SDR_RX, sdr->channel, init_frequency, tune_args_param);
        if(sdr->frequency_tune_args){
            SoapySDRKwargs_clear(&tune_args);
        }
        if(ret){
            fprintf(stderr, "SoapySDRDevice_setFrequency returned error %d (%s)\n", ret, SoapySDRDevice_lastError());
            exit(1);
        }
    }
    else if(sdr->frequency_tune_args){
        fprintf(stderr, "You must specify a frequency if specifying tune arguments.\n");
        exit(1);
    }
    sdr->frequency = SoapySDRDevice_getFrequency(sdr->device, SOAPY_SDR_RX, sdr->channel);
    fprintf(stderr, "Frequency set to %f MHz\n", (sdr->frequency / 1e6));

    // Set gain if specified
    if(!isnan(init_gain)){
        ret = SoapySDRDevice_setGain(sdr->device, SOAPY_SDR_RX, sdr->channel, init_gain);
        if(ret){
            fprintf(stderr, "SoapySDRDevice_setGain returned error %d (%s)\n", ret, SoapySDRDevice_lastError());
            exit(1);
        }
    }
    sdr->gain = SoapySDRDevice_getGain(sdr->device, SOAPY_SDR_RX, sdr->channel);
    fprintf(stderr, "Device gain set to %f dB\n", sdr->gain);

    // Set sample rate if specified
    if(!isnan(init_sample_rate)){
        ret = SoapySDRDevice_setSampleRate(sdr->device, SOAPY_SDR_RX, sdr->channel, init_sample_rate);
        if(ret){
            fprintf(stderr, "SoapySDRDevice_setSampleRate returned error %d (%s)\n", ret, SoapySDRDevice_lastError());
            exit(1);
        }
    }
    sdr->sample_rate = SoapySDRDevice_getSampleRate(sdr->device, SOAPY_SDR_RX, sdr->channel);
    fprintf(stderr, "Sample rate set to %f MHz\n", (sdr->sample_rate / 1e6));

    // Set bandwidth if specified
    if(!isnan(init_bandwidth)){
        ret = SoapySDRDevice_setBandwidth(sdr->device, SOAPY_SDR_RX, sdr->channel, init_bandwidth);
        if(ret){
            fprintf(stderr, "SoapySDRDevice_setBandwidth returned error %d (%s)\n", ret, SoapySDRDevice_lastError());
            exit(1);
        }
    }
    sdr->bandwidth = SoapySDRDevice_getBandwidth(sdr->device, SOAPY_SDR_RX, sdr->channel);
    fprintf(stderr, "Bandwidth set to %f MHz\n", (sdr->bandwidth / 1e6));

    // TODO: correction policies

  /*
  fprintf(stderr,"RTL freq = %u, tuner freq = %u\n",(unsigned)rtl_freq,(unsigned)tuner_freq);
  fprintf(stderr,"RTL tuner type %d\n",rtlsdr_get_tuner_type(sdr->device));
  int ngains = rtlsdr_get_tuner_gains(sdr->device,NULL);
  int gains[ngains];
  rtlsdr_get_tuner_gains(sdr->device,gains);
  fprintf(stderr,"Tuner gains:");
  for(int i=0; i < ngains; i++)
    fprintf(stderr," %d",gains[i]);
  fprintf(stderr,"\n");
  rtlsdr_set_freq_correction(sdr->device,0); // don't use theirs, only good to integer ppm
  rtlsdr_set_tuner_bandwidth(sdr->device, 0); // Auto bandwidth
  rtlsdr_set_agc_mode(sdr->device,0);

  if(AGC){
    rtlsdr_set_tuner_gain_mode(sdr->device,1); // manual gain mode (i.e., we do it)
    rtlsdr_set_tuner_gain(sdr->device,0);
    sdr->gain = 0;
    sdr->holdoff_counter = HOLDOFF_TIME;
  } else
    rtlsdr_set_tuner_gain_mode(sdr->device,0); // auto gain mode (i.e., the firmware does it)
  
  ret = rtlsdr_set_bias_tee(sdr->device,sdr->antenna_bias);

  rtlsdr_set_direct_sampling(sdr->device, 0); // That's for HF
  rtlsdr_set_offset_tuning(sdr->device,0); // Leave the DC spike for now
  if(sdr->samprate == 0){
    fprintf(stderr,"Select sample rate\n");
    exit(1);
  }
  ret = rtlsdr_set_sample_rate(sdr->device,(uint32_t)sdr->samprate);

  fprintf(stderr,"Set sample rate %'u Hz, IQ\n",sdr->samprate);

  // argv[optind] is presently just the USB device number, which is rather meaningless
  sdr->description = argv[optind];
  fprintf(stderr,"Rtlsdr handler %s: serial %llx\n",sdr->description,(long long unsigned)sdr->SN);
  ret = asprintf(&sdr->frequency_file,"%s/tune-rtlsdr.%llx",VARDIR,(unsigned long long)sdr->SN);
  if(ret == -1)
    exit(1);

  // Set up output sockets
  if(sdr->data_dest == NULL){
    // Construct from serial number
    int ret = asprintf(&sdr->data_dest,"rtlsdr-%016llx-pcm.local",(long long unsigned)sdr->SN);
    if(ret == -1)
      exit(1);
  }
  if(sdr->metadata_dest == NULL){
    // Construct from serial number
    int ret = asprintf(&sdr->metadata_dest,"rtlsdr-%016llx-status.local",(long long unsigned)sdr->SN);
    if(ret == -1)
      exit(1);
  }
  {
    // Start Avahi client that will maintain our mDNS registrations
    // Service name, if present, must be unique
    // Description, if present becomes TXT record if present
    char service_name[1024];
    snprintf(service_name,sizeof(service_name),"rtlsdr (%s)",sdr->metadata_dest);
    avahi_start(service_name,"_ka9q-ctl._udp",5006,sdr->metadata_dest,ElfHashString(sdr->metadata_dest),sdr->description);
    snprintf(service_name,sizeof(service_name),"rtlsdr (%s)",sdr->data_dest);
    avahi_start(service_name,"_rtp._udp",5004,sdr->data_dest,ElfHashString(sdr->data_dest),sdr->description);
  }
  {
    char iface[1024];
    resolve_mcast(sdr->data_dest,&sdr->output_data_dest_address,DEFAULT_RTP_PORT,iface,sizeof(iface));
    sdr->data_sock = connect_mcast(&sdr->output_data_dest_address,iface,RTP_ttl,IP_tos);
    if(sdr->data_sock == -1){
      fprintf(stderr,"Can't create multicast socket to %s: %s\n",sdr->data_dest,strerror(errno));
      exit(1);
    }
    socklen_t len = sizeof(sdr->output_data_source_address);
    getsockname(sdr->data_sock,(struct sockaddr *)&sdr->output_data_source_address,&len);
  
    resolve_mcast(sdr->metadata_dest,&sdr->output_metadata_dest_address,DEFAULT_STAT_PORT,iface,sizeof(iface));
    sdr->status_sock = connect_mcast(&sdr->output_metadata_dest_address,iface,Status_ttl,IP_tos);
    if(sdr->status_sock <= 0){
      fprintf(stderr,"Can't create multicast status socket to %s: %s\n",sdr->metadata_dest,strerror(errno));
      exit(1);
    }
    // Set up new control socket on port 5006
    sdr->nctl_sock = listen_mcast(&sdr->output_metadata_dest_address,iface);
    if(sdr->nctl_sock <= 0){
      fprintf(stderr,"Can't create multicast command socket from %s: %s\n",sdr->metadata_dest,strerror(errno));
      exit(1);
    }
  }
  sdr->rtp_type = IQ_PT8;

  {
    if(isnan(init_frequency)){
      // If not set on command line, load saved frequency
      FILE *fp = fopen(sdr->frequency_file,"r+");
      if(fp == NULL || fscanf(fp,"%lf",&init_frequency) < 0)
	fprintf(stderr,"Can't read stored freq from %s: %s\n",sdr->frequency_file,strerror(errno));
      else
	fprintf(stderr,"Using stored frequency %lf from tuner state file %s\n",init_frequency,sdr->frequency_file);
      if(fp != NULL)
	fclose(fp);
    }
  }
  if(isnan(init_frequency)){
    // Not set on command line, and not read from file. Use fallback to cover 2m
    init_frequency = 149e6; // Fallback default
    fprintf(stderr,"Fallback default frequency %'.3lf Hz\n",init_frequency);
  }
  fprintf(stderr,"Setting initial frequency %'.3lf Hz\n",init_frequency);
  set_correct_freq(sdr,init_frequency);
  time_t tt;
  time(&tt);
  if(sdr->rtp.ssrc == 0)
    sdr->rtp.ssrc = tt & 0xffffffff; // low 32 bits of clock time
    */

  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  
  if(sdr->status)
    pthread_create(&sdr->display_thread,NULL,display,sdr);

  pthread_create(&sdr->ncmd_thread,NULL,ncmd,sdr);
  //rtlsdr_reset_buffer(sdr->device);

  // Blocks until killed
  //rtlsdr_read_async(sdr->device,rx_callback,sdr,0,16*16384);

  exit(0);
}

// Thread to send metadata and process commands
void *ncmd(void *arg){
  // Send status, process commands
  pthread_setname("rtlsdr-cmd");
  assert(arg != NULL);
  struct sdrstate * const sdr = (struct sdrstate *)arg;
  if(sdr->status_sock == -1 || sdr->nctl_sock == -1) 
    return NULL; // Nothing to do

  /*
  int counter = 0;
  while(1){
    unsigned char buffer[Bufsize];
    memset(buffer,0,sizeof(buffer));
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100 ms

    if(setsockopt(sdr->nctl_sock,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv))){
      fprintf(stderr,"ncmd setsockopt: %s\n",strerror(errno));
      return NULL;
    }

    int length = recv(sdr->nctl_sock,buffer,sizeof(buffer),0);
    if(length > 0){
      // Parse entries
      unsigned char *cp = buffer;

      int cr = *cp++; // Command/response
      if(cr == 0)
	continue; // Ignore our own status messages
      sdr->commands++;
      decode_soapysdr_commands(sdr,cp,length-1);
    }      
    sdr->output_metadata_packets++;
    send_soapysdr_status(sdr,(counter == 0));
    if(counter-- <= 0)
      counter = 10;

    if(AGC)    
      do_rtlsdr_agc(sdr);
  }
  */

  return NULL;
}

// Status display thread
void *display(void *arg){
  assert(arg != NULL);
  struct sdrstate *sdr = (struct sdrstate *)arg;

  pthread_setname("rtlsdr-disp");

  fprintf(sdr->status,"               |-----Gains dB-- ---|      |----Levels dB --|           clips\n");
  fprintf(sdr->status,"Frequency      step LNA  mixer bband          RF   A/D   Out\n");
  fprintf(sdr->status,"Hz                                           dBFS  dBFS\n");

  /*
  off_t stat_point = ftello(sdr->status);
  // End lines with return when writing to terminal, newlines when writing to status file
  char   eol = stat_point == -1 ? '\r' : '\n';
  while(1){

    //    if(Frontend.sdr.isreal) Frontend.if_power *= 2; ??????????

    float powerdB = power2dB(sdr->power);

    if(stat_point != -1)
      fseeko(sdr->status,stat_point,SEEK_SET);
    
    fprintf(sdr->status,"%'-15.0lf%4d%4d%7d%6d%'12.1f%'6.1f%'6.1f%'16d    %c",
	    sdr->frequency,
	    0,0,0,0,
	    powerdB - sdr->gain / 10.,
	    powerdB,
	    powerdB,
	    sdr->clips,
	    eol);
    fflush(sdr->status);
    usleep(100000); // 10 Hz
  }
  */
  return NULL;
}

void decode_soapysdr_commands(struct sdrstate *sdr,unsigned char *buffer,int length){
  unsigned char *cp = buffer;


  while(cp - buffer < length){
    int ret __attribute__((unused)); // Won't be used when asserts are disabled
    enum status_type type = *cp++; // increment cp to length field

    /*
    if(type == EOL)
      break; // End of list
 
    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    
    switch(type){
    case EOL: // Shouldn't get here
      break;
    case COMMAND_TAG:
      sdr->command_tag = decode_int(cp,optlen);
      break;
    case CALIBRATE:
      sdr->calibration = decode_double(cp,optlen);
      break;
    case RADIO_FREQUENCY:
      if(!sdr->frequency_lock){
	double f = decode_double(cp,optlen);
	set_correct_freq(sdr,f);
      }
      break;
    default: // Ignore all others
      break;
    }
    cp += optlen;
    */
  }    
}  

void send_soapysdr_status(struct sdrstate *sdr,int full){
    /*
  unsigned char packet[2048],*bp;
  memset(packet,0,sizeof(packet));
  bp = packet;
  
  *bp++ = 0;   // Command/response = response
  
  encode_int32(&bp,COMMAND_TAG,sdr->command_tag);
  encode_int64(&bp,CMD_CNT,sdr->commands);
  
  encode_int64(&bp,GPS_TIME,gps_time_ns());

  if(sdr->description)
    encode_string(&bp,DESCRIPTION,sdr->description,strlen(sdr->description));

  // Source address we're using to send data
  encode_socket(&bp,OUTPUT_DATA_SOURCE_SOCKET,&sdr->output_data_source_address);
  // Where we're sending output
  encode_socket(&bp,OUTPUT_DATA_DEST_SOCKET,&sdr->output_data_dest_address);
  encode_int32(&bp,OUTPUT_SSRC,sdr->rtp.ssrc);
  encode_byte(&bp,OUTPUT_TTL,RTP_ttl);
  encode_int32(&bp,INPUT_SAMPRATE,sdr->samprate);
  encode_int64(&bp,OUTPUT_DATA_PACKETS,sdr->rtp.packets);
  encode_int64(&bp,OUTPUT_METADATA_PACKETS,sdr->output_metadata_packets);

  // Front end
  encode_double(&bp,OUTPUT_LEVEL,power2dB(sdr->power));
  encode_double(&bp,CALIBRATE,sdr->calibration);
  encode_double(&bp,DC_I_OFFSET,(double)sdr->DC / 32767. );
  encode_float(&bp,GAIN,(float)sdr->gain/10.);

  // Tuning
  encode_double(&bp,RADIO_FREQUENCY,sdr->frequency);
  encode_int32(&bp,LOCK,sdr->frequency_lock);

  encode_byte(&bp,DEMOD_TYPE,0); // actually LINEAR_MODE
  encode_int32(&bp,OUTPUT_SAMPRATE,sdr->samprate);
  encode_int32(&bp,OUTPUT_CHANNELS,2);
  encode_float(&bp,HIGH_EDGE,+0.47 * sdr->samprate);
  encode_float(&bp,LOW_EDGE,-0.47 * sdr->samprate); // Should look at the actual filter curves
  encode_int32(&bp,OUTPUT_BITS_PER_SAMPLE,8); // Always
  encode_byte(&bp,DIRECT_CONVERSION,1); // DC and 1/f spikes from RTL chip

  encode_eol(&bp);
  int len = bp - packet;
  assert(len < sizeof(packet));
  send(sdr->status_sock,packet,len,0);
  */
}

/*
// Callback called with incoming receiver data from A/D
void rx_callback(unsigned char *buf, uint32_t len, void *ctx){
  int samples = len;
  uint8_t *idp = (uint8_t *)buf;
  long long output_energy = 0;
  struct sdrstate * const sdr = (struct sdrstate *)ctx;
  
  struct rtp_header rtp;
  memset(&rtp,0,sizeof(rtp));
  rtp.version = RTP_VERS;
  rtp.type = sdr->rtp_type;
  rtp.ssrc = sdr->rtp.ssrc;

  while(samples > 0){
    int chunk = min(samples,sdr->blocksize);
    
    rtp.seq = sdr->rtp.seq++;
    rtp.timestamp = sdr->rtp.timestamp;
    
    unsigned char buffer[Bufsize];
    unsigned char *dp = buffer;
    
    dp = hton_rtp(dp,&rtp);
    
    for(int i=0;i < chunk; i += 1){
      // Samples are excess-128 integers, which are always positive
      // Convert to unbiased signed integers
      int s = *idp++;
      s -= 127;
      
#if REMOVE_DC
      samp_sum += s; // Accumulate average DC values
      
      s -= sdr->DC;   // remove smoothed DC offset (which can be fractional)
#endif
      output_energy += s * s;
      *dp++ = s;
    }

    if(send(sdr->data_sock,buffer,dp - buffer,0) == -1){
      fprintf(stderr,"send: %s\n",strerror(errno));
      //      usleep(100000); // inject a delay to avoid filling the log
    } else {
      sdr->rtp.packets++;
      sdr->rtp.bytes += (dp - buffer);
    }  
    sdr->rtp.timestamp += chunk/2; // complex samples      
    samples -= chunk;
  }
  // Update power estimate
  sdr->power = output_energy / (len * 256. * 256.);
  
#if REMOVE_DC
   sdr->DC += DC_alpha * (samp_sum - sdr->DC*sdr->blocksize);
#endif
}
void do_rtlsdr_agc(struct sdrstate *sdr){
  assert(sdr != NULL);
  if(!AGC)
    return; // Execute only in software AGC mode
    
  if(--sdr->holdoff_counter == 0){
    sdr->holdoff_counter = HOLDOFF_TIME;
    float powerdB = 10*log10f(sdr->power);
    if(powerdB > AGC_upper && sdr->gain > 0){
      sdr->gain -= 20;    // Reduce gain one step
    } else if(powerdB < AGC_lower){
      sdr->gain += 20;    // Increase one step
    } else
      return;
    // librtlsdr inverts its gain tables for some reason
    if(Verbose)
      printf("new tuner gain %.0f dB\n",(float)sdr->gain/10.);
    int r = rtlsdr_set_tuner_gain(sdr->device,sdr->gain);
    if(r != 0)
      printf("rtlsdr_set_tuner_gain returns %d\n",r);
  }
}
*/

static void closedown(int a){
  fprintf(stderr,"caught signal %'d: %s\n",a,strsignal(a));
  //rtlsdr_cancel_async(Device);

  if(a == SIGTERM) // sent by systemd when shutting down. Return success
    exit(0);
  exit(1);
}

