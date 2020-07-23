/*
 * Copyright (c) 2018 Dynamic Robotics Laboratory
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdio.h>
#include <thread>
#include "cassie_out_t.h"
#include "cassie_user_in_t.h"
#include "state_out_t.h"
#include "udp.h"

using namespace std::chrono_literals;

double k[10] = {200, 200, 200, 150, 20, 200, 200, 200, 150, 20};

void controller(cassie_user_in_t *cassie_user_in,
                const cassie_out_t *cassie_out) {
  static bool initial = true;
  static double q_d[10];
  double *u = cassie_user_in->torque;

  double q[10];
  q[0] = cassie_out->leftLeg.hipRollDrive.position;
  q[1] = cassie_out->leftLeg.hipYawDrive.position;
  q[2] = cassie_out->leftLeg.hipPitchDrive.position;
  q[3] = cassie_out->leftLeg.kneeDrive.position;
  q[4] = cassie_out->leftLeg.footDrive.position;
  q[5] = cassie_out->rightLeg.hipRollDrive.position;
  q[6] = cassie_out->rightLeg.hipYawDrive.position;
  q[7] = cassie_out->rightLeg.hipPitchDrive.position;
  q[8] = cassie_out->rightLeg.kneeDrive.position;
  q[9] = cassie_out->rightLeg.footDrive.position;

  double dq[10];
  dq[0] = cassie_out->leftLeg.hipRollDrive.velocity;
  dq[1] = cassie_out->leftLeg.hipYawDrive.velocity;
  dq[2] = cassie_out->leftLeg.hipPitchDrive.velocity;
  dq[3] = cassie_out->leftLeg.kneeDrive.velocity;
  dq[4] = cassie_out->leftLeg.footDrive.velocity;
  dq[5] = cassie_out->rightLeg.hipRollDrive.velocity;
  dq[6] = cassie_out->rightLeg.hipYawDrive.velocity;
  dq[7] = cassie_out->rightLeg.hipPitchDrive.velocity;
  dq[8] = cassie_out->rightLeg.kneeDrive.velocity;
  dq[9] = cassie_out->rightLeg.footDrive.velocity;

  if(initial) {
    initial = false;
    for(int i = 0; i < 10; i++) q_d[i] = q[i];
  }

  for(int i = 0; i < 10; i++) {
    u[i] = k[i] * (q_d[i] - q[i]) + k[i] / 5 * (0 - dq[i]);
  }
}

int main() {
  // Option variables and flags
  constexpr auto remote_addr_str = "127.0.0.1";
  constexpr auto remote_port_str = "25000";
  constexpr auto iface_addr_str = "0.0.0.0";
  constexpr auto iface_port_str = "25001";

  // Bind to network interface
  int sock = udp_init_client(remote_addr_str, remote_port_str, iface_addr_str,
                             iface_port_str);
  if(sock == -1) return -1;

  // Create packet input/output buffers
  constexpr auto recvlen = PACKET_HEADER_LEN + CASSIE_OUT_T_PACKED_LEN;
  constexpr auto sendlen = PACKET_HEADER_LEN + CASSIE_USER_IN_T_PACKED_LEN;
  unsigned char recvbuf[recvlen];
  unsigned char sendbuf[sendlen];

  // Separate input/output buffers into header and payload
  const unsigned char *header_in = recvbuf;
  const unsigned char *data_in = &recvbuf[PACKET_HEADER_LEN];
  unsigned char *header_out = sendbuf;
  unsigned char *data_out = &sendbuf[PACKET_HEADER_LEN];

  // Create standard input/output structs
  cassie_user_in_t cassie_user_in;
  cassie_out_t cassie_out;

  // Create header information struct
  packet_header_info_t header_info;

  // Prepare initial null command packet to start communication
  printf("Connecting to cassie...\n");
  bool received_data = false;

  int counter = 0;
  // Listen/respond loop
  while(true) {
    if(!received_data) {
      // Send null commands until the simulator responds
      ssize_t nbytes;
      do {
        send_packet(sock, sendbuf, sendlen, nullptr, 0);
        std::this_thread::sleep_for(1ms);
        nbytes = get_newest_packet(sock, recvbuf, recvlen, nullptr, nullptr);
      } while(recvlen != nbytes);
      received_data = true;
      printf("Connected!\n\n");
    } else {
      // Wait for a new packet
      wait_for_packet(sock, recvbuf, recvlen, nullptr, nullptr);
    }

    // Process incoming header and write outgoing header
    process_packet_header(&header_info, header_in, header_out);
    // printf("\033[F\033[Jdelay: %d, diff: %d\n", header_info.delay,
    //       header_info.seq_num_in_diff);

    // Unpack received data into cassie user input struct
    unpack_cassie_out_t(data_in, &cassie_out);

    // Run controller
    // Do nothing in this example
    double t = counter / 2000.0;
    printf("%f\n", t);
    controller(&cassie_user_in, &cassie_out);

    // Pack cassie out struct into outgoing packet
    pack_cassie_user_in_t(&cassie_user_in, data_out);

    // Send response
    send_packet(sock, sendbuf, sendlen, nullptr, 0);

    counter++;
  }
}
