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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Eigen/Eigen>

#define MAX_REC_BYTES 500

using namespace Eigen;

class Imagenex
{
	int Range[6];
	int Range_ind;
	int Rev;
	int Start_gain;
	int Train_angle;
	int Sector_width;
	int Step_size;
	int Step_length;

	int Switch_delay;
	int Freq;
	
    char ret_header[3];
	char head_ID;
	char serial_status;
	double head_pos;
	int step_dir;
	int range;
    char b89[2];
	int data_bytes;
    char data[500];
	char termination;
	char error_code[4];
	float av_b;
	

	public:
		
        int Data_points;

        char sRec[513],sCmd[29];
		
        VectorXf Raw_data;
        VectorXf sample;
		std::vector<float> correct;
		std::vector<float> corr_coef;
		std::vector<int> tab_peaks;
//		std::vector<float> sample;
		
		//int *sample, n_sample;
		//float *correct, *corr_coef;
		//int *tab_peaks,	n_peaks;
		
        float average, std_dev;
		
        Imagenex();
        Imagenex(const Imagenex &other);
		~Imagenex();
		
        void change_gain(int );
        void change_train_angle(int );
        void change_sector_width(int );
        void change_step_size(int );
        void change_step_length(int );
        void change_switch_delay(int );
        void change_frequency(int );
        void sonar_msg_build(void);
        void sonar_rec_parse(void);
        void change_range(int);
		
		
        int get_gain(void ){ return Start_gain;}
        int get_train_angle(void ){ return Train_angle;}
        int get_sector_width(void ){ return Sector_width;}
        int get_step_size(void ){ return Step_size;}
        int get_step_length(void ){ return Step_length;}
        int get_switch_delay(void ){ return Switch_delay;}
        double get_head_pos(void ){ return head_pos;}
        int get_step_dir(void ){ return step_dir;}
        int get_data_bytes(void ){ return data_bytes;}
        int get_range(void ){ return Range[Range_ind];}
        char get_data(int a ){ return data[a];}
        char get_sRec(int a ){ return sRec[a];}
		
        bool set_sample(VectorXf &s);
        VectorXf get_raw_data(void){ return Raw_data;}
		
		int sonar_rec_inspect(void);
		int max_index(void);
		int max_index(float *, int );
        int adjust_gain(float );

		template <class T> int max_index(T *x, int nelem)
		{
			int i=0,max_index=0;
		
			T max = 0;
			while(i < nelem)
			{
				if(x[i]>max)
				{
					max_index=i;
					max=x[i];
				}
				i++;
			}
			
			return(max_index);

		}

        template <class T> int max(std::vector<T> v, std::vector<int> & ind)
		{
            unsigned int i=0;
            ind.resize(0);
			
            if(v.size() > 0)
			{
                T max = v[0];
                ind.push_back(i);
                while(i < v.size())
				{
                    if(v[i] > max)
					{
                        ind.resize(0);
                        ind.push_back(i);
                        max = v[i];
					}
                    else if(v[i] == max)
                    {
                        ind.push_back(i);
                    }
					i++;
				}
			
                return(max);
			}
			else
			{
				return -1;
			}

		}
		
		template <class T> float mean(std::vector<T> x)
		{
			float sum = 0;
            for(unsigned int i = 0; i < x.size(); i++)
			{
				sum += x[i];
			}
	
			return sum/x.size();
		}
		

		
		template <class T> float standard_deviation(std::vector<T> data1, float average1)
		{
			float sum_err2=0;
	
            for(unsigned int i = 0; i < data1.size(); i++)
			{
				sum_err2 += pow((average1 - data1[i]),2);
			}
	
			float std_dev1 = sqrt((1.0/(data1.size()-1.0)) * sum_err2);
			return  std_dev1;
		}
		
		float get_simple_measure(int);
		float get_simple_measure_corr(int);
		
		template <class T> bool create_sample(T *sample_array, int n)
		{
			sample.resize(n);
			corr_coef.resize(500-n+1,0);

			for(int i = 0; i < n; i++)
			{
				this->sample[i] = sample_array[i];
			}
			if(sample.size() == n)
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		void compensate_loss(void );
		

		
};

float standard_deviation(VectorXf &vector);
VectorXf null_phase_filter(VectorXf &vector, int );
VectorXf cross_correlation(VectorXf &vector, VectorXf &sample);
VectorXf create_gaussian_sample(float a, float v, unsigned int n);
VectorXf find_peaks(VectorXf in, int min_dist, bool calc_peaks = true);
VectorXf make_histogram(VectorXf &input_data, unsigned short int nbins);
VectorXf make_histogram(VectorXf &input_data, VectorXf &bin_values, unsigned short int nbins);
VectorXf obstacle_interpretation(VectorXf current_read, VectorXf &previous_read, VectorXf &conf);
float similarity(VectorXf a, VectorXf b);
VectorXf get_window(VectorXf a, int i, int n);
