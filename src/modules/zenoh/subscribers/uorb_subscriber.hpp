/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uorb_subscriber.hpp
 *
 * Defines generic, templatized uORB over Zenoh / ROS2
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include "zenoh_subscriber.hpp"
#include <uORB/topics/input_rc.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <drivers/drv_hrt.h>
#include "rmw_attachment.h"

class uORB_Zenoh_Subscriber : public Zenoh_Subscriber
{
public:
	// d_instance: < (default if not in CSV) if we should create a new instance (safe), nonzero if we should use the 0 instance
	uORB_Zenoh_Subscriber(const orb_metadata *meta, const uint32_t *ops, int d_instance) :
		Zenoh_Subscriber(),
		_uorb_meta{meta},
		_cdr_ops(ops)
	{
		if (d_instance < 0) { // default=-1; allocate a new instance
			int instance;
			_uorb_pub_handle = orb_advertise_multi(_uorb_meta, nullptr, &instance);

		} else {
			_uorb_pub_handle = orb_advertise(_uorb_meta, nullptr);
		}
	};

	~uORB_Zenoh_Subscriber()
	{
		undeclare_subscriber();
		orb_unadvertise(_uorb_pub_handle);
	}

	// Update the uORB Subscription and broadcast a Zenoh ROS2 message
	void data_handler(const z_loaned_sample_t *sample)
	{
		_cb_in++;              // INSTRUMENTACAO (2026-09-24)
		_cb_t_in = hrt_absolute_time();

		char data[_uorb_meta->o_size];

		process_attachment(sample);
		const z_loaned_bytes_t *payload = z_sample_payload(sample);
		size_t len = z_bytes_len(payload);

		// Validate payload size to prevent stack overflow from untrusted input.
		// CDR payload = 4-byte header + serialized data, which should not exceed o_size + 4.
		const size_t max_payload_size = _uorb_meta->o_size + 4;

		if (len > max_payload_size || len < 4) {
			return;
		}

#if defined(Z_FEATURE_UNSTABLE_API)
		// Check if payload is contiguous so we can decode directly on that pointer
		z_view_slice_t view;

		if (z_bytes_get_contiguous_view(payload, &view) == Z_OK) {
			const uint8_t *ptr = z_slice_data(z_loan(view));

			dds_istream_t is = {.m_buffer = (unsigned char *)(ptr + 4), .m_size = static_cast<int>(len),
					    .m_index = 0, .m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_1
					   };
			dds_stream_read(&is, data, &dds_allocator, _cdr_ops);

		} else
#endif
		{
			unsigned char reassembled_payload[len];
			z_bytes_reader_t reader = z_bytes_get_reader(payload);
			z_bytes_reader_read(&reader, reassembled_payload, len);

			dds_istream_t is = {.m_buffer = &reassembled_payload[4], .m_size = static_cast<int>(len),
					    .m_index = 0, .m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_1
					   };
			dds_stream_read(&is, data, &dds_allocator, _cdr_ops);
		}

		_cb_deser++;  // INSTRUMENTACAO: CDR decodificado

		// As long as we don't have timesynchronization between Zenoh nodes
		// we've to manually set the timestamp
		fix_timestamp(data);

		// ORB_ID::input_rc needs additional timestamp fixup
		if (static_cast<ORB_ID>(_uorb_meta->o_id) == ORB_ID::input_rc) {
			memcpy(&data[8], data, sizeof(hrt_abstime));
		}

		_cb_pub++;  // INSTRUMENTACAO: prestes a entrar no orb_publish
		orb_publish(_uorb_meta, _uorb_pub_handle, &data);
		_cb_out++;  // INSTRUMENTACAO: orb_publish retornou
		_cb_t_out = hrt_absolute_time();
	};

	// Processa o attachment do rmw_zenoh (rmw_attachment.h). O rmw_zenoh anexa a
	// cada amostra um numero de sequencia por publisher, um timestamp e o GID que
	// identifica o publisher (design.md#publishers do rmw_zenoh).
	//
	// O numero de sequencia e a unica forma de saber, DO LADO DO SUBSCRIBER, que
	// uma amostra foi perdida no caminho: o zenoh em BEST_EFFORT nao avisa. Um
	// salto na sequencia significa amostra descartada entre o publisher e aqui.
	//
	// O GID e comparado para nao contar a sequencia de um publisher NOVO como
	// perda gigante: quando ele muda, a base e reiniciada.
	//
	// O campo `time` nao e usado: e o relogio do publisher, e nao ha sincronizacao
	// de tempo entre os nos -- e por isso que fix_timestamp() existe.
	void process_attachment(const z_loaned_sample_t *sample)
	{
		const z_loaned_bytes_t *att = z_sample_attachment(sample);

		if (att == nullptr || z_bytes_len(att) != RMW_ATTACHEMENT_SIZE) {
			_att_bad++;
			return;
		}

		RmwAttachment a;
		z_bytes_reader_t reader = z_bytes_get_reader(att);

		if (z_bytes_reader_read(&reader, (uint8_t *)&a, sizeof(a)) != sizeof(a)) {
			_att_bad++;
			return;
		}

		if ((unsigned)a.rmw_gid_size > RMW_GID_STORAGE_SIZE) {
			_att_bad++;
			return;
		}

		const bool same_pub = _seq_valid && (memcmp(_pub_gid, a.rmw_gid, RMW_GID_STORAGE_SIZE) == 0);

		if (!same_pub) {
			// Publisher novo (ou primeira amostra): rebase, sem contar perda
			if (_seq_valid) { _pub_changes++; }

			memcpy(_pub_gid, a.rmw_gid, RMW_GID_STORAGE_SIZE);
			_seq_valid = true;

		} else {
			const int64_t delta = a.sequence_number - _seq_last;

			if (delta > 1) {
				_seq_lost += (uint32_t)(delta - 1);
				_seq_gaps++;

			} else if (delta <= 0) {
				// Reordenada ou duplicada: nao mexe na base, para nao mascarar perda
				_seq_reorder++;
				return;
			}
		}

		_seq_last = a.sequence_number;
	}

	void fix_timestamp(char *data)
	{
		hrt_abstime now = hrt_absolute_time();
		memcpy(data, &now, sizeof(hrt_abstime));
	}

	void print()
	{
		Zenoh_Subscriber::print("uORB", _uorb_meta->o_name);
		print_counters();  // INSTRUMENTACAO (2026-09-24)
	}

	// INSTRUMENTACAO (2026-09-24): ver o cabecalho de read.c do zenoh-pico.
	// Roda na thread do shell (`zenoh status`), que continua viva com a read task
	// travada -- e por isso que o valor e legivel justamente na falha. Um par
	// in/out com delta 1 que NAO anda entre duas amostras localiza a moldura onde
	// a thread parou:
	//     in > deser  -> parou no dds_stream_read
	//     pub > out   -> parou dentro do orb_publish
	//     in == out   -> o callback nao e o culpado; olhar os contadores zp_*
	// REMOVER quando o bug estiver fechado.
	void print_counters()
	{
		const hrt_abstime now = hrt_absolute_time();
		PX4_INFO_RAW("   seq last:%lld perdidas:%lu saltos:%lu reord:%lu pub_novo:%lu att_ruim:%lu\n",
			     (long long)_seq_last, (unsigned long)_seq_lost, (unsigned long)_seq_gaps,
			     (unsigned long)_seq_reorder, (unsigned long)_pub_changes,
			     (unsigned long)_att_bad);
		PX4_INFO_RAW("   cb in:%lu deser:%lu pub:%lu out:%lu | ultimo in ha %llu ms, out ha %llu ms\n",
			     (unsigned long)_cb_in, (unsigned long)_cb_deser,
			     (unsigned long)_cb_pub, (unsigned long)_cb_out,
			     _cb_t_in ? (unsigned long long)((now - _cb_t_in) / 1000) : 0ULL,
			     _cb_t_out ? (unsigned long long)((now - _cb_t_out) / 1000) : 0ULL);
	}

protected:
	// Default payload-size function -- can specialize in derived class
	size_t get_payload_size()
	{
		return _uorb_meta->o_size;
	}

private:
	const orb_metadata *_uorb_meta;
	orb_advert_t _uorb_pub_handle;
	const uint32_t *_cdr_ops;

	// Rastreio de sequencia do attachment do rmw_zenoh
	int64_t _seq_last{0};
	uint32_t _seq_lost{0};      // amostras perdidas no caminho (soma dos saltos)
	uint32_t _seq_gaps{0};      // quantos saltos distintos
	uint32_t _seq_reorder{0};   // fora de ordem ou duplicadas
	uint32_t _pub_changes{0};   // trocas de publisher (GID diferente)
	uint32_t _att_bad{0};       // attachment ausente ou malformado
	bool _seq_valid{false};
	uint8_t _pub_gid[RMW_GID_STORAGE_SIZE] {};

	// INSTRUMENTACAO (2026-09-24) -- remover com o bug fechado
	volatile uint32_t _cb_in{0};
	volatile uint32_t _cb_deser{0};
	volatile uint32_t _cb_pub{0};
	volatile uint32_t _cb_out{0};
	volatile hrt_abstime _cb_t_in{0};
	volatile hrt_abstime _cb_t_out{0};
};
