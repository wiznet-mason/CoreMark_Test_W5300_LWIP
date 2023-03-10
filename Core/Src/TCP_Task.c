/* -----------------------------------------------------------------------------
 * Copyright (c) 2023 WIZnet Co., Ltd. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * -------------------------------------------------------------------------- */
 
#include "TCP_Task.h"
#include "string.h"
#include "cmsis_os.h"

#define SOCKET_NUM 0
#define TARGET_PORT 5000
#define LOCAL_PORT 6000

extern osSemaphoreId_t send_semHandle;
extern osSemaphoreId_t coremark_start_semHandle;


/* Network */
extern uint8_t mac[6];
static ip_addr_t g_ip;
static ip_addr_t g_mask;
static ip_addr_t g_gateway;
static ip_addr_t g_target_ip;

/* LWIP */
static struct netif g_netif;
static struct tcp_pcb *tcp_client_pcb = NULL;

static uint8_t eth_tx_buf[ETHERNET_BUF_MAX_SIZE];
static uint8_t pack[ETHERNET_MTU];

/* LWIP */
static err_t tcp_callback_connected(void *arg, struct tcp_pcb *pcb_new, err_t err);
static err_t tcp_callback_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t tcp_callback_received(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t tcp_callback_poll(void *arg, struct tcp_pcb *tpcb);
static void tcp_callback_error(void *arg, err_t err);
static void tcp_client_close(struct tcp_pcb *tpcb);


void tcp_task_main (void *argument) {
  int ret;
  uint32_t pack_len = 0;
  struct pbuf *p = NULL;

  IP4_ADDR(&g_ip, 192, 168, 2, 120);
  IP4_ADDR(&g_mask, 255, 255, 255, 0);
  IP4_ADDR(&g_gateway, 192, 168, 2, 1);
  IP4_ADDR(&g_target_ip, 192, 168, 2, 100); //  <<replace host ip;

  printf("%s\r\n", __func__);
  memset(eth_tx_buf, NULL, ETHERNET_BUF_MAX_SIZE);

  netif_add(&g_netif, &g_ip, &g_mask, &g_gateway, NULL, netif_initialize, netif_input);
  g_netif.name[0] = 'e';
  g_netif.name[1] = '0';
  
  // Assign callbacks for link and status
  netif_set_link_callback(&g_netif, netif_link_callback);
  netif_set_status_callback(&g_netif, netif_status_callback);
  
  // MACRAW socket open
  ret = socket(SOCKET_NUM, Sn_MR_MACRAW, LOCAL_PORT, 0x00);
  if (ret < 0)
    printf(" MACRAW socket open failed\n");
  
  netif_set_link_up(&g_netif);
  netif_set_up(&g_netif);

  tcp_client_pcb = (struct tcp_pcb *)tcp_new();
  printf("tcp_client_pcb = %d, 0x%08X\r\n", tcp_client_pcb, tcp_client_pcb);
	if (tcp_client_pcb == NULL)
	{
    printf("tcp_client_pcb == NULL\r\n");
    while(1);
	}
  
  ret = tcp_connect(tcp_client_pcb, &g_target_ip, TARGET_PORT, tcp_callback_connected);
  if (ret == ERR_OK)
  {
    printf(" Connect [%d.%d.%d.%d:%d]\n",
            ip4_addr1(&tcp_client_pcb->remote_ip),
            ip4_addr2(&tcp_client_pcb->remote_ip),
            ip4_addr3(&tcp_client_pcb->remote_ip),
            ip4_addr4(&tcp_client_pcb->remote_ip),
            tcp_client_pcb->remote_port);                            
  }
  else
  {
    printf(" Connect failed err = %d\r\n", ret);
    /* Deallocate the pcb */
    memp_free(MEMP_TCP_PCB, tcp_client_pcb);
    while(1);
  }

  while (1)
  {
    getsockopt(SOCKET_NUM, SO_RECVBUF, &pack_len);
    if (pack_len > 0)
    {
      pack_len = recv_lwip(SOCKET_NUM, (uint8_t *)pack, pack_len);
      if (pack_len)
      {
        p = pbuf_alloc(PBUF_RAW, pack_len, PBUF_POOL);
        pbuf_take(p, pack, pack_len);
      }
      else
      {
        printf(" No packet received\n");
      }

      if (pack_len && p != NULL)
      {
        LINK_STATS_INC(link.recv);
        if (g_netif.input(p, &g_netif) != ERR_OK)
        {
          printf("g_netif.input(p, &g_netif) != ERR_OK\r\n");
          pbuf_free(p);
        }
      }
    }
    sys_check_timeouts();
    osDelay(1);
  }
}

void send_task_main(void *argument)
{
  uint32_t send_count = 0;

  while (1)
  {
    osSemaphoreAcquire(send_semHandle, osWaitForever);
    send_count++;
    sprintf(eth_tx_buf, "send count = %d\r\n\0", send_count);
    tcp_write(tcp_client_pcb, eth_tx_buf, strlen(eth_tx_buf), 0);
  }
}

static err_t tcp_callback_connected(void *arg, struct tcp_pcb *pcb_new, err_t err)
{
  LWIP_UNUSED_ARG(arg);
  printf("[%s] err = %d\r\n", __func__, err);
  
  if (err != ERR_OK) //error when connect to the server
    return err;
  
  tcp_setprio(pcb_new, TCP_PRIO_NORMAL); //set priority for the client pcb

  tcp_arg (pcb_new, 0); //no argument is used
  tcp_err (pcb_new, tcp_callback_error); //register error callback
  tcp_sent(pcb_new, tcp_callback_sent); //register send callback
  tcp_recv(pcb_new, tcp_callback_received);  //register receive callback
  tcp_poll(pcb_new, tcp_callback_poll, 0); //register poll callback

  osSemaphoreRelease(send_semHandle);
  osSemaphoreRelease(coremark_start_semHandle);
  return ERR_OK;
}

static err_t tcp_callback_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(tpcb);
  LWIP_UNUSED_ARG(len);

  //osSemaphoreRelease(send_semHandle);
  return ERR_OK;
}

static err_t tcp_callback_received(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  err_t ret_err;

  if (p == NULL) //pbuf is null when session is closed
  {
    tcp_client_close(tpcb);
    printf("Client socket closed => 0x%x\n", tpcb);
    ret_err = ERR_OK;
  }
  else if (err != ERR_OK) //ERR_ABRT is returned when called tcp_abort
  {
    tcp_recved(tpcb, p->tot_len); //advertise window size
    pbuf_free(p); //free pbuf
    ret_err = err;
  }
  else //receiving data
  {
    tcp_recved(tpcb, p->tot_len); 
    printf("%.*s", p->len, p->payload);
    pbuf_free(p); //free pbuf
    ret_err = ERR_OK;
    osSemaphoreRelease(send_semHandle);
  }
  return ret_err;

}


static void tcp_callback_error(void *arg, err_t err)
{
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);
}


static err_t tcp_callback_poll(void *arg, struct tcp_pcb *tpcb)
{
  return ERR_OK;
}

static void tcp_client_close(struct tcp_pcb *tpcb)
{
  /* Clear callback functions */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  tcp_close(tpcb); // close connection
}


