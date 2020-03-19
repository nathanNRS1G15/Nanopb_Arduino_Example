#include "Arduino.h"

#include "packet.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

/************************ Classes ************************/

/**
 * Class to encapsulate the Capture submessage nanopb encoding.
 * 
 * @author nrs1g15@soton.ac.uk
 */
class ProtoCapture
{ 
public:
  /**
   * Initialise or re-initialise the class instance.
   * 
   * @param p_adc_readings Pointer to ADC values to encode.
   * @param adc_channels The number of passed ADC values to encode.
   * 
   * @return void
   */
  void init(int32_t *p_adc_readings, const size_t adc_channels);

  /**
   * Get the class's associated Capture struct.
   * 
   * @return Capture* Pointer to the associated Capture struct.
   */
  Capture *get_capture();
  
private:
  static bool encode_callback(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg);

  Capture capture;
  int32_t *p_adc_readings;
  size_t channelsAmt;
};

/**
 * Class to encapsulate the Wrapper message nanopb encoding.
 * 
 * @author nrs1g15@soton.ac.uk
 */
class ProtoWrapper
{
public:
  /**
   * Wrapper constructor
   * 
   * @param p_captures Pointer to ProtoCaptures array to encode.
   * @param captures_amt The number of passed ProtoCaptures to encode.
   * @param sequence_number Sequence number to encode.
   * @param timestamp Timestamp to encode.
   */
  ProtoWrapper(ProtoCapture *p_captures, const size_t captures_amt, uint32_t sequence_number, uint32_t timestamp);

  /**
   * Encode all the passed constructor parameters to the buffer.
   * 
   * @param p_buffer Pointer to the buffer to encode to.
   * @param buffer_len The length of the passed buffer.
   * 
   * @return size_t 0 if failed to encode or the number of bytes the encoding consists of.
   */
  size_t encode(byte *p_buffer, size_t buffer_len);

private:
  static bool encode_callback(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg);

  Wrapper wrapper;
  ProtoCapture *p_captures;
  size_t capturesAmt;
  
};

void ProtoCapture::init(int32_t *p_adc_readings, const size_t adc_channels)  
{
  this->capture = Capture_init_default;
  this->capture.channelVoltage.funcs.encode = &this->encode_callback;  
  this->capture.channelVoltage.arg = this;
  this->p_adc_readings = p_adc_readings;
  this->channelsAmt = adc_channels;
}

bool ProtoCapture::encode_callback(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg)  
{
  ProtoCapture *self = (ProtoCapture *) *arg;
  
  for (size_t i = 0; i < self->channelsAmt; i++)  
  {
    if (!pb_encode_tag_for_field(stream, field))
    {
      goto error;
    } 
    if (!pb_encode_varint(stream, self->p_adc_readings[i]))
    {
      goto error;
    }
  }

  return true;
error:
  return false;
}

Capture *ProtoCapture::get_capture()  
{
  return &this->capture; 
}

ProtoWrapper::ProtoWrapper(ProtoCapture *p_captures, const size_t captures_amt, uint32_t sequence_number, uint32_t timestamp)  
{ 
  this->p_captures = p_captures;
  this->capturesAmt = captures_amt;

  this->wrapper = Wrapper_init_default;
  this->wrapper.has_timeStamp = true;
  this->wrapper.timeStamp  = timestamp;
  this->wrapper.has_sequenceNumber = true;
  this->wrapper.sequenceNumber = sequence_number;

  this->wrapper.capture.funcs.encode = &this->encode_callback;
  this->wrapper.capture.arg = this;
}

size_t ProtoWrapper::encode(byte *p_buffer, size_t buffer_len)  
{
  pb_ostream_t stream = pb_ostream_from_buffer(p_buffer, buffer_len);  
  bool status = pb_encode(&stream, Wrapper_fields, &this->wrapper);  
  return (status == true) ? stream.bytes_written : 0; 
}

bool ProtoWrapper::encode_callback(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg)
{
  ProtoWrapper *self = (ProtoWrapper *) *arg; 

  for (size_t i = 0; i < self->capturesAmt; i++) 
  {
    Capture *capture = self->p_captures[i].get_capture();  
    if (!pb_encode_tag_for_field(stream, field))
    {
      goto error; 
    }
    if (!pb_encode_submessage(stream, Capture_fields, capture)) 
    {
      goto error; 
    }
  }
  return true;  
error:
  return false; 
}

/************************ Compile Time Variables ************************/

const size_t buffer_len = 1024;
byte buffer[buffer_len];

const size_t adc_readings_amt = 8;
const size_t captures_amt = 10;
int32_t adc_readings[captures_amt][adc_readings_amt];

ProtoCapture captures[captures_amt];

uint32_t sequence_number = 1337;


/************************ Setup ************************/
void setup()
{
  Serial.begin(115200);

  while (!Serial);

}

/************************ Loop ************************/
void loop()
{
  int32_t dummyAdcReading = 0;
  
  for (size_t i = 0; i < captures_amt; i++)
  {
    captures[i].init(adc_readings[i], adc_readings_amt);
    for (size_t j = 0; j < adc_readings_amt; j++)
    {
      adc_readings[i][j] = dummyAdcReading++;
    }
  }
  ProtoWrapper wrapper = ProtoWrapper(captures, captures_amt, sequence_number, millis());
  size_t bytesWritten = wrapper.encode(buffer, buffer_len);

  Serial.print(">");
  Serial.write(buffer, bytesWritten);
  
  while (true);
}
