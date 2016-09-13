#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <time.h>
#include <sys/time.h>

/*
 *TODO: add size limit checks to enhance security
 *TODO: add more info to  session IDs.
 *TODO: make session ID forming a seperate func and use in all jsons
 *TODO: add optional logging
 *TODO: support URL encoding of numbers and special charas
 *TODO: replace system() with code integration for speedup
 *
 */

#define STD_PUB_COMMAND_TEMPLATE  "mosquitto_pub -t %s -m '%s' -q 1 -r"

static unsigned char debug=0;


typedef struct queryNode queryNode;

struct queryNode{
  char* key;
  char* value;
  queryNode* next;
};

typedef enum{
  MESSAGE_TYPE_STD=0,
}message_t;

typedef enum{
  EXIT_RETVAL_SUCCESS=0,
  EXIT_RETVAL_ID_UNKNOWN=-1,
  EXIT_RETVAL_ID_EMPTY_REQ=-2,
  EXIT_RETVAL_ID_ALLOC_FAILED=-3,
  EXIT_RETVAL_ID_UNHANDLED=-4,
  EXIT_RETVAL_NODE_ALLOCATED=-5,
  EXIT_RETVAL_NODE_ALLOC_FAILED=-6,
  EXIT_RETVAL_NODE_UNHANDLED=-7,
  EXIT_RETVAL_PUB_ALLOC_FAILED_QUERY=-8,
  EXIT_RETVAL_PUB_ALLOC_FAILED_COMMAND=-9,
  EXIT_RETVAL_PUB_UNHANDLED=-10,
}exitRetval_t;

/*global speed up variables*/
static char* REQUEST_URI;
static char* QUERY_STRING;
static int gQueryCount;

/*global memory allocation flags*/
static char *gaf_thingID=NULL;
static queryNode *gaf_queryNodeHead=NULL;

static int get_thingID(char **thingID);
static void process_get_thingID_retval(int retval);
static int get_query_count(void);
static int alloc_query_nodes(queryNode* queryNodeHead);
static void process_alloc_query_nodes_retval(int retval);
static void free_query_node(queryNode* queryNodeHead);
static int parse_query_string(queryNode* queryNodeHead); 
//static void traverse_query_string(queryNode* queryNodeHead);
static int mqtt_pub(char* thingID,queryNode* queryNodeHead,message_t messageType);
static void process_mqtt_pub_retval(int retval);
static int clean_exit(exitRetval_t);

int main(int argc, char *argv[])
{
  char *thingID=NULL;

  /*debug and test*/
  if (NULL!=getenv("DEBUG")){debug=1;}

  /*init global*/
  REQUEST_URI=getenv("REQUEST_URI");
  
  QUERY_STRING=getenv("QUERY_STRING");
  if(NULL==QUERY_STRING){
    QUERY_STRING="";
  }
  
  gQueryCount=get_query_count();

  process_get_thingID_retval(get_thingID(&thingID));

  queryNode queryNodeHead;
  gaf_queryNodeHead=&queryNodeHead;
  memset(&queryNodeHead,0,sizeof(queryNode));
  process_alloc_query_nodes_retval(alloc_query_nodes(&queryNodeHead));

  parse_query_string(&queryNodeHead);
  //traverse_query_string(&queryNodeHead);


  process_mqtt_pub_retval(mqtt_pub(thingID,&queryNodeHead,MESSAGE_TYPE_STD));
  clean_exit(EXIT_RETVAL_SUCCESS);
  return 0;
}


/*extract thing ID from requestURI*/
static int get_thingID(char **thingID)
{
  char *scriptName=basename(getenv("SCRIPT_NAME")); /*allow better deployment options*/
  
    char *tempID=strstr(REQUEST_URI,scriptName);
    tempID+=strlen(scriptName);
    if('\0'!=*tempID) /*test case - dont add a topic*/
    {
      tempID=strtok(tempID+1,"?"); 
    }
    else{
      return -1;    
    }
    if(NULL!=tempID){
      *thingID=(char*)calloc(1,sizeof(char)*(1+strlen(tempID)));
      if(NULL!=*thingID){
        gaf_thingID=*thingID;
        strcpy(*thingID,tempID);
      }
      else{
        return -3;
      }
    }
    else{
      return -1;
    }
    return 0;
}

/*process the outcome of parsing thingID*/
static void process_get_thingID_retval(int retval)
{

  switch(retval){
    case -1:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 400 Bad request\n\n");
      printf("{\"error\":{\"code\":\"ERR_ID_THING_UNKNOWN\",\"reason\":\"thing unknown\"}}"); /*test case: request ending in / */
      clean_exit(EXIT_RETVAL_ID_UNKNOWN); /*if there is no thing, then there is no thang*/ 
      break;
    case -2:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_ID_REQUEST_STRING_EMPTY\",\"reason\":\"request string is empty\"}}"); 
      clean_exit(EXIT_RETVAL_ID_EMPTY_REQ); 
      break;
    case -3:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_ID_ALLOC_FAILED\",\"reason\":\"ID alloc failed\"}}"); 
      clean_exit(EXIT_RETVAL_ID_ALLOC_FAILED); 
      break;
    case 0:
      break;
    default:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_ID_UNHANDLED_CASE\",\"reason\":\"unhandled scenario\"}}"); 
      clean_exit(EXIT_RETVAL_ID_UNHANDLED); 
      break;
  }

}

/*find number of queries*/
static int get_query_count(void)
{

  if (NULL!=QUERY_STRING){
    int queryCount_eql=0;
    int queryCount_amb=0;
    unsigned int queryLength=strlen(QUERY_STRING);

    if(0==queryLength)
      return 0;

    if(('&'==QUERY_STRING[queryLength-1])||\
       ('='==QUERY_STRING[queryLength-1])){ /*possible malformed query*/
      return 0;
    }

    for(int i=0;i<=queryLength;i++){
      if ('='==QUERY_STRING[i]) queryCount_eql++;
    }
    for(int i=0;i<=queryLength;i++){
      if ('&'==QUERY_STRING[i]) queryCount_amb++;
    }
    if(1==queryCount_eql){
      return 1;
    }
    else{
      if(queryCount_eql != (queryCount_amb+1)) return 0; /*we dont support non key=val queries*/
      else {
        return queryCount_eql;
      }
    }
  }
  else{
    return 0;
  }
}


/*create a linked list to store the queries*/
static int alloc_query_nodes(queryNode* queryNodeHead)
{
  if(0==gQueryCount)
    return 0;


  int queryCount = gQueryCount-1; /*we already gave head*/

  if(NULL!=queryNodeHead->next){
    return -1; /*already initialized*/
  }
  else{
    queryNode* newNode;
    while(queryCount>0){
      newNode=(queryNode*)calloc(1,sizeof(queryNode));
      if(NULL!=newNode){
        queryNodeHead->next=newNode;
        queryNodeHead=newNode;
        queryCount--;
      }
      else{
        return -2; /*calloc failed*/
      }
    }
    return 0;
  }
}

/*process the outcome of query nodes allocation*/
static void process_alloc_query_nodes_retval(int retval)
{
  switch(retval){
    case -1 :
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_NODE_ALREADY_ALLOCATED\",\"reason\":\"node already allocated\"}}");
      clean_exit(EXIT_RETVAL_NODE_ALLOCATED);  
      break;
    case -2 : 
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_NODE_ALLOC_FAILED\",\"reason\":\"failed to alloc node\"}}");
      clean_exit(EXIT_RETVAL_NODE_ALLOC_FAILED);  
      break;
    case 0 :
      break;
    default:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_NODE_UNHANDLED_CASE\",\"reason\":\"unhandled scenario\"}}"); 
      clean_exit(EXIT_RETVAL_NODE_UNHANDLED); 
      break;
  }


}

/*free the query nodes LL*/
static void free_query_node(queryNode* queryNodeHead)
{
  queryNode* queryNodeHandle=queryNodeHead->next;
  queryNode* queryNodeTemp;

  if(0==gQueryCount)
    return;

  free(queryNodeHead->key);
  free(queryNodeHead->value);
  while(NULL!=queryNodeHandle){
    queryNodeTemp=queryNodeHandle->next;
    free(queryNodeHandle->key);
    free(queryNodeHandle->value);
    free(queryNodeHandle);
    queryNodeHandle=queryNodeTemp;
  }
  free(queryNodeHandle);
  queryNodeHead->next=NULL;
}

/*parse the query string and store it in the queryNode LL*/
static int parse_query_string(queryNode* queryNodeHead)
{
  char *queryPtr=NULL,*query=NULL;
  char *key, *value;

  if(0!=gQueryCount){
    queryPtr=(char*)calloc(1,sizeof(char)*(strlen(QUERY_STRING)+1));
    query=queryPtr;
    strcpy(query,QUERY_STRING);
  }
  else{
    return 0;
  }

  for (int i=0;i<gQueryCount;i++){
    key=strtok(query,"=");
    queryNodeHead->key=(char*)malloc(sizeof(char) * (strlen(key)+1));
    strcpy(queryNodeHead->key,key);
    query+=strlen(key)+sizeof(char);

    value=strtok(query,"&");
    queryNodeHead->value=(char*)malloc(sizeof(char) * (strlen(value)+1));
    strcpy(queryNodeHead->value,value);
    query+=strlen(value)+sizeof(char);

    queryNodeHead=queryNodeHead->next;
  }
  free((void*)queryPtr);
  return 0;
}

#if 0
/*a local test function to traverse the LL and print its contents*/
static void traverse_query_string(queryNode* queryNodeHead)
{
  while(NULL!=queryNodeHead){
    //printf("%s:%s\n",queryNodeHead->key,queryNodeHead->value);
    queryNodeHead=queryNodeHead->next;
  } 
}
#endif

/*publish the reconstructed queryString to topic-thingID*/
static int mqtt_pub(char* thingID,queryNode* queryNodeHead,message_t messageType)
{
  char *query,*command;
  unsigned int allocQueryFlag=0;
  time_t epochTime;
  struct timeval tv;
  
  char *commandTemplate;
  switch(messageType)
  {
    case (MESSAGE_TYPE_STD):
      commandTemplate=STD_PUB_COMMAND_TEMPLATE;
      break;
    default:
      break;
  }


  if(gQueryCount>0){/*frame Json from query*/
    unsigned int queryLength= sizeof(char)*(strlen(QUERY_STRING)+(5*gQueryCount)+10);
    
    query=(char*)malloc(queryLength);
    if(NULL==query){
      return -1;
    }
    else{
      allocQueryFlag=1;
    }

    command=(char*)malloc(queryLength+(sizeof(char)*(strlen(thingID)+strlen(commandTemplate)+10)));
    if(NULL==command){
      free(query);
      return -2;
    }
    else{
    }

    strcpy(query,"{\"");
    while(NULL!=queryNodeHead->next){
      strcat(query,queryNodeHead->key);
      strcat(query,"\":\"");
      strcat(query,queryNodeHead->value);
      strcat(query,"\",\"");
      queryNodeHead=queryNodeHead->next;
    }
    strcat(query,queryNodeHead->key);
    strcat(query,"\":\"");
    strcat(query,queryNodeHead->value);
    strcat(query,"\"}");
  }
  else{
    query="{}";
    allocQueryFlag=0;
    command=(char*)calloc(1,(sizeof(char)*(strlen(thingID)+strlen(commandTemplate)+10)));
    if(NULL==command){
      free(query);
      return -2;
    }
  }

  switch(messageType)
  {
    case (MESSAGE_TYPE_STD):
      sprintf(command, STD_PUB_COMMAND_TEMPLATE,thingID,query);
      break;
    default:
      break;
  }

  epochTime=time(NULL);
  char *timeStr=asctime(gmtime(&epochTime));
  timeStr[strlen(timeStr) - 1] = 0;

  gettimeofday(&tv, NULL);
  unsigned long long epochMilSec = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;

  char transactionString[64];
  snprintf(transactionString,(sizeof(char)*63),"%s:%s-%llx",getenv("REMOTE_ADDR"),getenv("REMOTE_PORT"),epochMilSec);
    
  int retval=system(command);
  retval=WEXITSTATUS(retval);

  switch(retval){
    case (0):
      printf("Content-type: application/json; charset=utf-8\n\n");
      printf("{\"with\":{\"topic\":\"%s\",\"created\":\"%s\",\"content\":%s,\"session\":\"%s\"}}",thingID,timeStr,query,transactionString);
    break;
    default:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 400 Bad request\n\n");
      printf("{\"error\":{\"code\":\"ERR_PUB_FAILED\",\"reason\":\"pubError(%d)\"}}",retval); /*test case: request ending in / */
    break;
  }
  
  if (0!=allocQueryFlag) free(query);
  free(command);
  
  return(0);
}

static void process_mqtt_pub_retval(int retval)
{
 switch(retval){
  case -1:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_PUB_ALLOC_FAILED\",\"reason\":\"failed to alloc query\"}}");
      clean_exit(EXIT_RETVAL_PUB_ALLOC_FAILED_QUERY);  
    break;
  case -2:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_PUB_ALLOC_FAILED\",\"reason\":\"failed to alloc command\"}}");
      clean_exit(EXIT_RETVAL_PUB_ALLOC_FAILED_COMMAND);  
    break;
  case 0:
    break;
  default:
      printf("Content-type: application/json; charset=utf-8\n");
      printf("status: 500 Internal Server Error\n\n");
      printf("{\"error\":{\"code\":\"ERR_PUB_UNHANDLED_CASE\",\"reason\":\"unhandled scenario\"}}"); 
      clean_exit(EXIT_RETVAL_PUB_UNHANDLED); 
    break;
 } 
}

static int clean_exit(exitRetval_t retVal)
{
  if(0!=gaf_thingID){
    free((void*)gaf_thingID);
  }

  if(NULL!=gaf_queryNodeHead){
    free_query_node(gaf_queryNodeHead);
  }
  exit(retVal);
}
