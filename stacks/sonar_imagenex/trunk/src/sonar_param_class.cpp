/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: João Sousa on 06/06/2013
*********************************************************************/

#include "sonar_param_class.h"
#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

Imagenex::Imagenex()
{
    Range[0] = 5;
    Range[1] = 10;
    Range[2] = 20;
    Range[3] = 30;
    Range[4] = 40;
    Range[5] = 50;
	Range_ind = 1;
    Rev = 0;
    Start_gain = 3;
    Train_angle = 0;
    Sector_width = 110;
    Step_size = 1;
    Step_length = 10;
    Data_points = 500;
    Switch_delay = 0;
    Freq = 850;
    Raw_data.resize(data_bytes);
    sample.resize(0);
}

Imagenex::Imagenex(const Imagenex &other)
{
    Range[0] = 5;
    Range[1] = 10;
    Range[2] = 20;
    Range[3] = 30;
    Range[4] = 40;
    Range[5] = 50;

    Range_ind = other.Range_ind;
    Rev = other.Rev;
    Start_gain = other.Start_gain;
    Train_angle = other.Train_angle;
    Sector_width = other.Sector_width;
    Step_size = other.Step_size;
    Step_length = other.Step_length;
    Data_points = other.Data_points;
    Switch_delay = other.Switch_delay;
    Freq = other.Freq;
    Raw_data.resize(data_bytes);
    sample.resize(0);
}

Imagenex::~Imagenex()
{
	
}

void Imagenex::change_gain(int a)
{
    Start_gain = a;
    sonar_msg_build();
}

void Imagenex::change_train_angle(int a)
{
    Train_angle = a;
    sonar_msg_build();
}

void Imagenex::change_sector_width(int a)
{
	float s = 0.666666667 * (float)a;
    Sector_width = (int)s;
    sonar_msg_build();
}

void Imagenex::change_step_size(int a)
{
    Step_size = a;
    sonar_msg_build();
}

void Imagenex::change_step_length(int a)
{
    Step_length = a;
    sonar_msg_build();
}
	
void Imagenex::change_switch_delay(int a)
{
    Switch_delay = a;
    sonar_msg_build();
}

void Imagenex::change_frequency(int a)
{
    Freq = a;
    sonar_msg_build();
}

void Imagenex::change_range(int r)
{
	int i = 0,temp = -1;
	for(i = 0; i < 6; i++)
	{
		if(Range[i] == r)
		{
			temp = i;
		}
	}
	if(temp == -1)
	{
		Range_ind = 0;
	}
	else
	{
		Range_ind = temp;
	}
    sonar_msg_build();
}

bool Imagenex::set_sample(VectorXf &s)
{
    sample.resize(s.rows());
    sample = s;

    if(s.rows())
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Imagenex::sonar_msg_build(void)
{
	sCmd[0]=0xFE;
	sCmd[1]=0x44;
	sCmd[2]=0x10;
	
	sCmd[3]=Range[Range_ind];

	sCmd[4]=0x00;
	
	if(Rev == 0) sCmd[5]=0x00;
	else sCmd[5]=0x40;
	
	sCmd[6]=0x00;
	
	sCmd[7]=0x00;
	
    if(Start_gain < 0) sCmd[8]=0;
    else if(Start_gain > 40) sCmd[8]=40;
    else sCmd[8]=Start_gain;

    sCmd[9]=0x00;
    sCmd[10]=20;

    int ang=Train_angle;
    if(ang<-210) ang=-210;
    if(ang>210) ang=210;
    sCmd[11]=(ang+210)/3;

    int sec=Sector_width;
    if(sec<0) sec=0;
    else if(sec>360) sec=sec%360;
	sCmd[12]=sec;
	
	int step=Step_size;
	switch(step)
	{
		case 0:
			sCmd[13]=0;
			break;
		case 3:
			sCmd[13]=1;
			break;
		case 6:
			sCmd[13]=2;
			break;
		default:
			sCmd[13]=1;
	}
	
	int sl=Step_length;
	if(sl<1) sl=1;
	if(sl>255) sl=255;
	sCmd[14]=sl;
	
	sCmd[15]=0x00;
	sCmd[16]=0x00;
	sCmd[17]=0x00;
	sCmd[18]=0x00;
	
	if(Data_points != 250 && Data_points != 500) sCmd[19]=50;
	else sCmd[19]=(int)Data_points/10; 
	
	sCmd[20]=0x00;
	sCmd[21]=0x00;
	sCmd[22]=0x00;
	sCmd[23]=0x00;
	
	int sd=Switch_delay;
	if(sd<0) sd=0;
	else if(sd>255) sd=127;
	else if(sd==253) sd=127;
	else sd/=2;
	sCmd[24]=sd;
	
	if(Freq != 850) sCmd[25]=0;
	else sCmd[25]=135; 
	
	sCmd[26]=0xFD;
	
    data_bytes = 500;
	
}

void Imagenex::sonar_rec_parse(void)
{
	strncpy(ret_header,sRec,3);
	
	head_ID=sRec[3];
	serial_status=sRec[4];
	
	int head_pos_H, head_pos_L;
	head_pos_H=(sRec[6] & 0x3E)>>1;
	head_pos_L=(((sRec[6] & 0x01)<<7) | (sRec[5] & 0x7F));
	
	head_pos= (double)(((head_pos_H << 8) | head_pos_L ));
	head_pos= (head_pos- 1400.0 )*0.15;
	
	step_dir=(sRec[6] & 0x40)>>6;
	range=(int)sRec[7];
	
	strncpy(b89,sRec+8,2);
	
	int data_byte, data_byte_H, data_byte_L;
	data_byte_H=(sRec[11] & 0x7E)>>1;
	data_byte_L=(((sRec[11] & 0x01)<<7) | (sRec[10] & 0x7F));
	data_byte= (data_byte_H << 8) | data_byte_L;
	data_bytes=(int)data_byte;
    //data_bytes=(int)500;
	
	memcpy(data,sRec+12,500);
	
    termination=sRec[data_bytes-1];

    if(Raw_data.rows() != data_bytes)
    {
        Raw_data.resize(data_bytes);
    }

    for(int f= 0; f < data_bytes; f++)
    {
        Raw_data(f) = (int)data[f];
    }
}

int Imagenex::sonar_rec_inspect(void)
{	
	int e = 0;
	if(strncmp(ret_header,"IMX",3)!=0 && strncmp(ret_header,"IGX",3)!=0)
	{
		error_code[0]=65;
		e++;
	}
	if(head_ID!=0x10)
	{
		error_code[1]=65;
		e++;
	}
	if(b89[0]!=0x00 || b89[1]!=0x00)
	{
		error_code[2]=65;
		e++;
	}
	if(data_bytes!=500 && data_bytes!=252)
	{
		error_code[3]=65;
		e++;
	}
	return(e);
}
	
	
int Imagenex::max_index(void)
{
	int i=0,max_index=0,max=0;
	
	while(i<MAX_REC_BYTES && data[i]!=0xFC)
	{
		if((int)data[i]>max)
		{
			max_index=i;
			max=(int)data[i];
		}
		i++;
	}
	
	return(max_index);
}

int Imagenex::adjust_gain(float value)
{
	if(value > 0.9*Range[Range_ind])
	{
		if(Range_ind < 5)
		{
			(this->Range_ind)++;
			return 1;
		}
	}
	else if(value < 0.9*Range[Range_ind - 1] && Range_ind > 0)
	{
		(this->Range_ind)--;
		return 1;
	}
	
	return 0;
}
	
void Imagenex::compensate_loss(void)
{
//	float val;
	
//	for(int i = 0 ; i < data_bytes ; i++)
//	{
//		//data[i] = (uint8_t)( (float)data[i] / (70.0-10.0*log10((float)i+1))) ;
//		val = (uint8_t)( (70.0-10.0*log((float)i+1))) ;
//		(this->correct[i])/=val;
//	}
}


//float Imagenex::get_simple_measure(int n)
//{
//    float av, dev, dist = -1;
//    int max_index = 0;

//    if(data_bytes)
//    {
//        av = this->mean();
//        dev = this->standard_deviation();

//        for(int i = 0; i < data_bytes; i++)
//        {
//            if((int)data[i] > (n * dev + av))
//            {
//                if((int)data[max_index] < (int)data[i])
//                {
//                    max_index = i;
//                }
//            }
//        }


//        dist = (max_index) * float(range) / float(data_bytes);
//    }

//    return dist;
//}

float standard_deviation(VectorXf &vector)
{
    float sum_err2=0;

    for(int i = 0; i < vector.rows(); i++)
    {
        sum_err2 += pow((vector.mean() - vector(i)),2);
    }

    float std_dev = sqrt((1.0/(vector.rows() - 1.0)) * sum_err2);
    return  std_dev;
}


VectorXf null_phase_filter(VectorXf &vector, int w)
{
    if(w % 2 == 0) w++;

    int w2 = (int)floor(w/2.0);

    float temp_sum;

    VectorXf filtered = vector;

    for (int i = 0; i < vector.rows(); i++)
    {
        temp_sum = 0;

        for(int f = -w2; f <= w2; f++)
        {
            if( i+f < 0)
            {
                temp_sum += vector(0);
            }
            else if(i+f >= vector.rows())
            {
                temp_sum += vector(vector.rows()-1);
            }
            else
            {
                temp_sum += vector(i+f);
            }
        }
        filtered(i) = (temp_sum / (float)w);
    }
    return filtered;
}

VectorXf cross_correlation(VectorXf &vector, VectorXf &sample)
{
    float av_a = vector.mean();
    float av_b = sample.mean();

    VectorXf corr_coef;

    float da = 0, db = 0;
    for(unsigned int i = 0; i < vector.rows(); i++)
    {
        da += (vector(i) - av_a) * (vector(i) - av_a);
    }
    for(unsigned int i = 0; i < sample.rows(); i++)
    {
        db += (sample(i) - av_b) * (sample(i) - av_b);
    }
    float denom = sqrt(da*db);

    int j,c = floor(sample.rows() / 2.0);
    float sxy;
    corr_coef.resize(vector.rows() - sample.rows()+1+c);

    if(denom != 0)
    {
        for(unsigned int d = 0; d <= (vector.rows() - sample.rows()) ; d++)
        {
            //printf("oh %d\n",d);
            sxy = 0;
            for(unsigned int i=0; i < sample.rows(); i++)
            {
                j = i + d;

                sxy += (vector(j) - av_a) * (sample(i) - av_b);

            }

            corr_coef(d+c) = sxy / denom;
        }
    }
    return corr_coef;
}


VectorXf create_gaussian_sample(float a, float v, unsigned int n)
{
    if(n < 5)
    {
        n = 5;
    }

    VectorXf sample;
    sample.resize(n);

    for(unsigned int i = 0; i < n; i++)
    {
        sample(i) = a * exp(-pow(((float)i - (float)n/2.0) / 100.0,2.0) / (2.0* v*v));
        printf("%f\n", sample(i));
    }
    return sample;
}

VectorXf find_peaks(VectorXf in, int min_dist, bool calc_peaks)
{
    VectorXf tab_peaks, tab_peaks_index, tab_peaks_filtrada;
    tab_peaks.resize(0);

    std::vector<float> peaks;
    std::vector<uint16_t> peaks_index;

    if(in.rows() > 0)
    {
        if(!calc_peaks)
        {
            float mmm = in.maxCoeff();
            for(int i = 0; i < in.rows(); i++)
            {
                in(i) = mmm - in(i);
            }
        }

        //cout << in.transpose() << endl;
        if(in.rows() > 2)
        {
            if(in(1) < in(0))
            {
                peaks.push_back(in(0));
                peaks_index.push_back(0);
            }
            for( int i = 1; i < in.rows() - 1 ; i++)
            {
                if(in(i) > in(i - 1) && in(i) >= in(i + 1))
                {
                    peaks.push_back(in(i));
                    peaks_index.push_back(i);
                }
            }
            if(in(in.rows() - 2) < in(in.rows() - 1))
            {
                peaks.push_back(in(in.rows() - 1));
                peaks_index.push_back(in.rows() - 1);
            }
        }
        else if(in.rows() == 2)
        {
            if(in(0) > in(1))
            {
                peaks.push_back(in(0));
                peaks_index.push_back(0);
            }
            else
            {
                peaks.push_back(in(1));
                peaks_index.push_back(1);
            }
        }
        else if(in.rows() == 1)
        {
            peaks.push_back(in(0));
            peaks_index.push_back(0);
        }

        tab_peaks.resize(peaks.size());
        tab_peaks_index.resize(peaks_index.size());

        for(unsigned int i = 0; i < peaks.size(); i++)
        {
            tab_peaks(i) = peaks[i];
            tab_peaks_index(i) = peaks_index[i];
        }

        //std::cout << " old " << in.transpose() << std::endl;

        int max_ind;
        int i = 0;

        //cout << "treta " << max_tabpeaks_ind << endl << tab_peaks << endl;

        std::vector<int> h,l;

        if(tab_peaks.rows() > 0)
        {
            tab_peaks.maxCoeff(&max_ind);
            h.push_back(max_ind);

            int last_peak = max_ind;
            i = last_peak + 1;
            while(i < tab_peaks_index.rows())
            {
                if(tab_peaks_index(i) - tab_peaks_index(last_peak) > min_dist)
                {
                    h.push_back(i);
                    last_peak = i;
                }
                i++;
            }
            i = max_ind - 1;
            while(i >= 0)
            {
                if(tab_peaks_index(last_peak) - tab_peaks_index(i) > min_dist)
                {
                    l.push_back(i);
                    last_peak = i;
                }
                i--;
            }

            int c = 0;
            tab_peaks_filtrada.resize(l.size()+h.size());
            for(int i = (int)l.size()-1; i >= 0; i--)
            {
                tab_peaks_filtrada(c++) = tab_peaks_index(l[i]);
            }
            for(unsigned int i = 0; i < h.size(); i++)
            {
                tab_peaks_filtrada(c++) = tab_peaks_index(h[i]);
            }
        }
    }


    /*float av = in.mean();
    float dev = standard_deviation(in);
    int nelem = in.rows() + 1 - sample.size();
    //printf("n%f %f\n",av,dev);


    for(int i = 0 ; i < nelem ; i++)
    {
        if(in[i] > (float)sd * dev + av)
        {
            tab[i] = 1;
        }
        else
        {
            tab[i] = 0;
        }
        //printf("%d-%d\n",i,tab[i]);
    }*/

//    int ind1 = 0,ind2 = 0, ind, in_peak = 0;

//    for(int i = 0 ; i < nelem ; i++)
//    {
//        if(tab[i] != 0 && !in_peak)
//        {
//            ind1 = i;
//            ind2 = ind1;
//            in_peak = 1;
//        }
//        else if(tab[i] == 0 && in_peak)
//        {
//            in_peak=0;
//            ind2 = i;
//            ind = floor((ind1 + ind2) / 2.0);

//            //(this->n_peaks++);

//            //printf("ind %d\n",0.2/(float)Range[Range_ind]*(float)data_bytes);
//            //if(ind > 0.2/(float)Range[Range_ind]*(float)data_bytes)
//        //	{
//                this->tab_peaks.push_back(ind);
//        //	}
//        }
//    }


//    /*for(int i = 0 ; i < n_peaks ; i++)
//    {
//        printf("%d |",tab_peaks[i]);
//    }
//    printf("\n");*/

    return tab_peaks_filtrada;
}

VectorXf make_histogram(VectorXf &input_data, unsigned short int nbins)
{
    VectorXf hist, classes;
    hist.resize(nbins);
    hist.setZero(hist.rows(),hist.cols());
    classes.resize(nbins);

    if(nbins > 0 && input_data.rows() > 0)
    {
        float min_value = input_data.minCoeff(), max_value = input_data.maxCoeff();
        float bin_width =  (max_value - min_value) / nbins;

        for(int i = 0; i < input_data.rows(); i++)
        {
            int ind = floor((input_data(i) - min_value) / bin_width);
            if (ind < 0)
            {
                ind = 0;
            }
            else if(ind >= nbins)
            {
                ind = nbins - 1;
            }
            hist(ind)++;
        }
    }

    return hist;
}

VectorXf make_histogram(VectorXf &input_data, VectorXf &bin_values, unsigned short int nbins)
{
    VectorXf hist;
    hist.resize(nbins);
    hist.setZero(hist.rows(),hist.cols());
    bin_values.resize(nbins);

    if(nbins > 0 && input_data.rows() > 0)
    {
        float min_value = input_data.minCoeff(), max_value = input_data.maxCoeff();
        float bin_width =  (max_value - min_value) / nbins;
        float first_bin = min_value + bin_width / 2;

        for(int i = 0; i < nbins; i++)
        {
            bin_values(i) = first_bin + i * bin_width;
        }

        for(int i = 0; i < input_data.rows(); i++)
        {
            int ind = floor((input_data(i) - min_value) / bin_width);
            if (ind < 0)
            {
                ind = 0;
            }
            else if(ind >= nbins)
            {
                ind = nbins - 1;
            }
            hist(ind)++;
        }
    }
    return hist;
}

VectorXf obstacle_interpretation(VectorXf current_read, VectorXf &previous_read, VectorXf &conf)
{
    VectorXf h,c,m, peaks,x, peaks_filt;

    x = current_read;                                   // store current value

    // (1)
    // h = histogram, c = bin values
    h = make_histogram(current_read, c, 20);            // make histogram of the current read

    // (2)
    // m = current read filtered
    m = null_phase_filter(h, 5);                        // filter current read

    // (4)
    //find first local minima
    peaks = find_peaks(m, 1, false);                    // find minima

    // (5)
    // use the index as threshould
    cout << "peaks\n" << peaks << "\nc\n" << c << endl;
    if(peaks.rows() > 0)
    {
        for(int i = 0; i < current_read.rows(); i++)
        {
            if(x(i) <= c(peaks(0)))
            {
                x(i) = 0;
            }
        }
    }

    // (10)
    // m = current read filtered
    m = null_phase_filter(m, 3);                        // filter current read

    // (11)
    // find local maxima,  separated by 20 samples
    peaks_filt = find_peaks(x, 20);

    // (12)
    //cout << "peaks_filt" << peaks_filt << endl;
    float c_sim = similarity(x, previous_read);

    // (13)
    conf.resize(peaks_filt.rows());

    VectorXf a,b;

    // (14)
    //cout << "peaks \n" << peaks_filt << endl;
    for(int i = 0; i < peaks_filt.rows(); i++)
    {
        // (15)
        //cout << "peak(i) " << peaks_filt(i) << endl << " i " << i << endl;
        a = get_window(x, peaks_filt(i), 40);
        // (16)
        //cout << "a\n" << a << endl;
        b = get_window(previous_read, peaks_filt(i), 40);
        //cout << "b\n" << b << endl;
        // (16)
        conf(i) = similarity(a, b) / c_sim;
    }

    previous_read = current_read;
    return peaks_filt;
}

float similarity(VectorXf a, VectorXf b)
{
    //cout << a.rows() << " " << b.rows() << endl;

    float n = a.dot(b),d = (a.norm() * b.norm());

    if(d < -0.001 || d > 0.0001)
    {
        return (n / d);
    }
    else
    {
        return 0;
    }
}

VectorXf get_window(VectorXf a,  int i,  int n)
{
    VectorXf ret = a, y;
    ret.setZero();

    int half_window = floor(n / 2.0);

    if(n < a.rows())
    {
        if ( i >= 0 && i < a.rows() && half_window >= 0)
        {
            if ( i + half_window < a.rows() && (i - half_window) >= 0)
            {
                cout << "caso1\n";
                //cout << a.block(158,0,2,1) << endl;
                //return a.block(i,0,n,1);
                y = a.block(i-half_window,0,n + 1, 1);
                ret.block(i-half_window,0,n + 1, 1) = y;
            }
            else if ( i + half_window >= a.rows() && (i - half_window) >= 0)
            {
                cout << "caso2\n";
                //return a.block(i,0,a.rows() - 1 - i, 1 );
                y = a.block(i - half_window, 0, a.rows() + half_window - i, 1 );
                ret.block(i - half_window, 0, a.rows() + half_window - i, 1) = y;
            }
            else if ( i + half_window < a.rows() && (i - half_window) < 0)
            {
                cout << "caso3\n";
                //return a.block(i,0,a.rows() - 1 - i, 1 );
                y = a.block(0, 0, half_window + i, 1 );
                ret.block(0, 0, half_window + i, 1) = y;
            }
        }
        cout << "caso4\n";
    }
    else
    {
        cout << "caso5\n";
        y = a;
    }


    cout << "y\n" << y << endl;

    return ret;
}
