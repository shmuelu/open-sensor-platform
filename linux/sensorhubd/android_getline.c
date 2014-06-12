/*
 * Copyright 2012, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/** \file
 * Android doesn't come with an implementation of getline by default so we
 * need to provide this. 
 */


#include <stdio.h>
#include <stdlib.h>

 size_t getline(char **lineptr, size_t *n, FILE *stream) {
 	char *bufptr = NULL;
 	char *p = bufptr;
 	size_t diff;
 	size_t size;
 	int c;

 	if (lineptr == NULL) {
 		return -1;
 	}
 	if (stream == NULL) {
 		return -1;
 	}
 	if (n == NULL) {
 		return -1;
 	}
 	bufptr = *lineptr;
 	if(n)
 		size = *n;

 	c = fgetc(stream);
 	if (c == EOF) {
 		return -1;
 	}
 	if (bufptr == NULL) {
 		bufptr = (char*)malloc(128);
 		if (bufptr == NULL) {
 			return -1;
 		}
 		size = 128;
 	}
 	p = bufptr;
 	while(c != EOF) {
 		if ((p - bufptr) > (size - 1)) {
 			size = size + 128;
 			diff = p - bufptr;
 			bufptr = (char*)realloc(bufptr, size);
 			if (bufptr == NULL) {
 				return -1;
 			}
 			p = bufptr + diff;
 		}
 		*p++ = c;
 		if (c == '\n') {
 			break;
 		}
 		c = fgetc(stream);
 	}

 	*p++ = '\0';
 	*lineptr = bufptr;
 	if(n)
 		*n = size;

 	return p - bufptr - 1;
 }
