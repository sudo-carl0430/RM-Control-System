#include "message_center.h"
#include "stdlib.h"
#include "string.h"

static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL};

static void CheckName(char *name)
{
    size_t len = strlen(name);
    if (len >= MAX_TOPIC_NAME_LEN)
    {
        while (1)
            ; // 进入这里说明话题名超出长度限制
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        while (1)
            ; 
    }
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    CheckName(name);
    Publisher_t *node = &message_center;
    while (node->next_topic_node) 
    {
        node = node->next_topic_node;            
        if (strcmp(node->topic_name, name) == 0) 
        {
            CheckLen(data_len, node->data_len);
            node->pub_registered_flag = 1;
            return node;
        }
    } 
    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->pub_registered_flag = 1;
    return node->next_topic_node;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    Publisher_t *pub = PubRegister(name, data_len); 
    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    memset(ret, 0, sizeof(Subscriber_t));
    ret->data_len = data_len; 
    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    { 
        ret->queue[i] = malloc(data_len);
    }
    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        return ret;
    }
    Subscriber_t *sub = pub->first_subs; 
    while (sub->next_subs_queue)         
    {
        sub = sub->next_subs_queue; 
    }
    sub->next_subs_queue = ret; 
    return ret;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    if (sub->temp_size == 0)
    {
        return 0;
    }
    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx++) % QUEUE_SIZE; 
    sub->temp_size--;                                 
    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    static Subscriber_t *iter;
    iter = pub->first_subs; 
    while (iter)
    {
        if (iter->temp_size == QUEUE_SIZE) 
        {
            iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
            iter->temp_size--; 
        }
        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
        iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE; 
        iter->temp_size++;                                  

        iter = iter->next_subs_queue; 
    }
    return 1;
}

