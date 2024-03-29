// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "particle_t.h"

static int __particle_t_hash_computed;
static int64_t __particle_t_hash;

int64_t __particle_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __particle_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__particle_t_get_hash;
    (void) cp;

    int64_t hash = (int64_t)0x18016676f01f14e8LL
         + __pose_xyt_t_hash_recursive(&cp)
         + __pose_xyt_t_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __particle_t_get_hash(void)
{
    if (!__particle_t_hash_computed) {
        __particle_t_hash = __particle_t_hash_recursive(NULL);
        __particle_t_hash_computed = 1;
    }

    return __particle_t_hash;
}

int __particle_t_encode_array(void *buf, int offset, int maxlen, const particle_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __pose_xyt_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].pose), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __pose_xyt_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].parent_pose), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].weight), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int particle_t_encode(void *buf, int offset, int maxlen, const particle_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __particle_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __particle_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __particle_t_encoded_array_size(const particle_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __pose_xyt_t_encoded_array_size(&(p[element].pose), 1);

        size += __pose_xyt_t_encoded_array_size(&(p[element].parent_pose), 1);

        size += __double_encoded_array_size(&(p[element].weight), 1);

    }
    return size;
}

int particle_t_encoded_size(const particle_t *p)
{
    return 8 + __particle_t_encoded_array_size(p, 1);
}

size_t particle_t_struct_size(void)
{
    return sizeof(particle_t);
}

int particle_t_num_fields(void)
{
    return 3;
}

int particle_t_get_field(const particle_t *p, int i, lcm_field_t *f)
{
    if (0 > i || i >= particle_t_num_fields())
        return 1;
    
    switch (i) {
    
        case 0: {
            /* pose_xyt_t */
            f->name = "pose";
            f->type = LCM_FIELD_USER_TYPE;
            f->typestr = "pose_xyt_t";
            f->num_dim = 0;
            f->data = (void *) &p->pose;
            return 0;
        }
        
        case 1: {
            /* pose_xyt_t */
            f->name = "parent_pose";
            f->type = LCM_FIELD_USER_TYPE;
            f->typestr = "pose_xyt_t";
            f->num_dim = 0;
            f->data = (void *) &p->parent_pose;
            return 0;
        }
        
        case 2: {
            f->name = "weight";
            f->type = LCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void *) &p->weight;
            return 0;
        }
        
        default:
            return 1;
    }
}

const lcm_type_info_t *particle_t_get_type_info(void)
{
    static int init = 0;
    static lcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (lcm_encode_t) particle_t_encode;
        typeinfo.decode         = (lcm_decode_t) particle_t_decode;
        typeinfo.decode_cleanup = (lcm_decode_cleanup_t) particle_t_decode_cleanup;
        typeinfo.encoded_size   = (lcm_encoded_size_t) particle_t_encoded_size;
        typeinfo.struct_size    = (lcm_struct_size_t)  particle_t_struct_size;
        typeinfo.num_fields     = (lcm_num_fields_t) particle_t_num_fields;
        typeinfo.get_field      = (lcm_get_field_t) particle_t_get_field;
        typeinfo.get_hash       = (lcm_get_hash_t) __particle_t_get_hash;
    }
    
    return &typeinfo;
}
int __particle_t_decode_array(const void *buf, int offset, int maxlen, particle_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __pose_xyt_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].pose), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __pose_xyt_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].parent_pose), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].weight), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __particle_t_decode_array_cleanup(particle_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __pose_xyt_t_decode_array_cleanup(&(p[element].pose), 1);

        __pose_xyt_t_decode_array_cleanup(&(p[element].parent_pose), 1);

        __double_decode_array_cleanup(&(p[element].weight), 1);

    }
    return 0;
}

int particle_t_decode(const void *buf, int offset, int maxlen, particle_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __particle_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __particle_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int particle_t_decode_cleanup(particle_t *p)
{
    return __particle_t_decode_array_cleanup(p, 1);
}

int __particle_t_clone_array(const particle_t *p, particle_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __pose_xyt_t_clone_array(&(p[element].pose), &(q[element].pose), 1);

        __pose_xyt_t_clone_array(&(p[element].parent_pose), &(q[element].parent_pose), 1);

        __double_clone_array(&(p[element].weight), &(q[element].weight), 1);

    }
    return 0;
}

particle_t *particle_t_copy(const particle_t *p)
{
    particle_t *q = (particle_t*) malloc(sizeof(particle_t));
    __particle_t_clone_array(p, q, 1);
    return q;
}

void particle_t_destroy(particle_t *p)
{
    __particle_t_decode_array_cleanup(p, 1);
    free(p);
}

int particle_t_publish(lcm_t *lc, const char *channel, const particle_t *p)
{
      int max_data_size = particle_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = particle_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _particle_t_subscription_t {
    particle_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void particle_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    particle_t p;
    memset(&p, 0, sizeof(particle_t));
    status = particle_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding particle_t!!!\n", status);
        return;
    }

    particle_t_subscription_t *h = (particle_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    particle_t_decode_cleanup (&p);
}

particle_t_subscription_t* particle_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    particle_t_handler_t f, void *userdata)
{
    particle_t_subscription_t *n = (particle_t_subscription_t*)
                       malloc(sizeof(particle_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 particle_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg particle_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int particle_t_subscription_set_queue_capacity (particle_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int particle_t_unsubscribe(lcm_t *lcm, particle_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe particle_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

