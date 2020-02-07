/*
 * ewma.c
 *
 *  Created on: 2 f�vr. 2016
 *      Author: Patrick
 */

#include "ewma.h"

void reset_ewma( ewma_handler * handler )
{
    handler->mean = 0.0;
}

void reset_ewma_with_non_zero_data( ewma_handler * handler, double initial_mean )
{
    handler->mean = initial_mean;
    //handler->mean = 0.0;
}

float process_ewma( ewma_handler * handler, double data )
{
	handler->mean = data**handler->alpha+(1-*handler->alpha)*handler->mean;
    return handler->mean;
}
