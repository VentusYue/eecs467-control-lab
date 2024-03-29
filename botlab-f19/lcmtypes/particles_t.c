// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "particles_t.h"

static int __particles_t_hash_computed;
static int64_t __particles_t_hash;

int64_t __particles_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __particles_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__particles_t_get_hash;
    (void) cp;

    int64_t hash = (int64_t)0xc48afb43c4cd5590LL
         + __int64_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __particle_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __particles_t_get_hash(void)
{
    if (!__particles_t_hash_computed) {
        __particles_t_hash = __particles_t_hash_recursive(NULL);
        __particles_t_hash_computed = 1;
    }

    return __particles_t_hash;
}

int __particles_t_encode_array(void *buf, int offset, int maxlen, const particles_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].num_particles), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __particle_t_encode_array(buf, offset + pos, maxlen - pos, p[element].particles, p[element].num_particles);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int particles_t_encode(void *buf, int offset, int maxlen, const particles_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __particles_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __particles_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __particles_t_encoded_array_size(const particles_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

        size += __int32_t_encoded_array_size(&(p[element].num_particles), 1);

        size += __particle_t_encoded_array_size(p[element].particles, p[element].num_particles);

    }
    return size;
}

int particles_t_encoded_size(const particles_t *p)
{
    return 8 + __particles_t_encoded_array_size(p, 1);
}

size_t particles_t_struct_size(void)
{
    return sizeof(particles_t);
}

int particles_t_num_fields(void)
{
    return 3;
}

int particles_t_get_field(const particles_t *p, int i, lcm_field_t *f)
{
    if (0 > i || i >= particles_t_num_fields())
        return 1;
    
    switch (i) {
    
        case 0: {
            f->name = "utime";
            f->type = LCM_FIELD_INT64_T;
            f->typestr = "int64_t";
            f->num_dim = 0;
            f->data = (void *) &p->utime;
            return 0;
        }
        
        case 1: {
            f->name = "num_particles";
            f->type = LCM_FIELD_INT32_T;
            f->typestr = "int32_t";
            f->num_dim = 0;
            f->data = (void *) &p->num_particles;
            return 0;
        }
        
        case 2: {
            /* particle_t */
            f->name = "particles";
            f->type = LCM_FIELD_USER_TYPE;
            f->typestr = "particle_t";
            f->num_dim = 1;
            f->dim_size[0] = p->num_particles;
            f->dim_is_variable[0] = 1;
            f->data = (void *) &p->particles;
            return 0;
        }
        
        default:
            return 1;
    }
}

const lcm_type_info_t *particles_t_get_type_info(void)
{
    static int init = 0;
    static lcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (lcm_encode_t) particles_t_encode;
        typeinfo.decode         = (lcm_decode_t) particles_t_decode;
        typeinfo.decode_cleanup = (lcm_decode_cleanup_t) particles_t_decode_cleanup;
        typeinfo.encoded_size   = (lcm_encoded_size_t) particles_t_encoded_size;
        typeinfo.struct_size    = (lcm_struct_size_t)  particles_t_struct_size;
        typeinfo.num_fields     = (lcm_num_fields_t) particles_t_num_fields;
        typeinfo.get_field      = (lcm_get_field_t) particles_t_get_field;
        typeinfo.get_hash       = (lcm_get_hash_t) __particles_t_get_hash;
    }
    
    return &typeinfo;
}
int __particles_t_decode_array(const void *buf, int offset, int maxlen, particles_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].num_particles), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element].particles = (particle_t*) lcm_malloc(sizeof(particle_t) * p[element].num_particles);
        thislen = __particle_t_decode_array(buf, offset + pos, maxlen - pos, p[element].particles, p[element].num_particles);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __particles_t_decode_array_cleanup(particles_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

        __int32_t_decode_array_cleanup(&(p[element].num_particles), 1);

        __particle_t_decode_array_cleanup(p[element].particles, p[element].num_particles);
        if (p[element].particles) free(p[element].particles);

    }
    return 0;
}

int particles_t_decode(const void *buf, int offset, int maxlen, particles_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __particles_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __particles_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int particles_t_decode_cleanup(particles_t *p)
{
    return __particles_t_decode_array_cleanup(p, 1);
}

int __particles_t_clone_array(const particles_t *p, particles_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

        __int32_t_clone_array(&(p[element].num_particles), &(q[element].num_particles), 1);

        q[element].particles = (particle_t*) lcm_malloc(sizeof(particle_t) * q[element].num_particles);
        __particle_t_clone_array(p[element].particles, q[element].particles, p[element].num_particles);

    }
    return 0;
}

particles_t *particles_t_copy(const particles_t *p)
{
    particles_t *q = (particles_t*) malloc(sizeof(particles_t));
    __particles_t_clone_array(p, q, 1);
    return q;
}

void particles_t_destroy(particles_t *p)
{
    __particles_t_decode_array_cleanup(p, 1);
    free(p);
}

int particles_t_publish(lcm_t *lc, const char *channel, const particles_t *p)
{
      int max_data_size = particles_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = particles_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _particles_t_subscription_t {
    particles_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void particles_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    particles_t p;
    memset(&p, 0, sizeof(particles_t));
    status = particles_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding particles_t!!!\n", status);
        return;
    }

    particles_t_subscription_t *h = (particles_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    particles_t_decode_cleanup (&p);
}

particles_t_subscription_t* particles_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    particles_t_handler_t f, void *userdata)
{
    particles_t_subscription_t *n = (particles_t_subscription_t*)
                       malloc(sizeof(particles_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 particles_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg particles_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int particles_t_subscription_set_queue_capacity (particles_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int particles_t_unsubscribe(lcm_t *lcm, particles_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe particles_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

