#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <string.h>
#include <unistd.h>

int main() {
    snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *hw_params;
    int err;

    char *pcm_device = "hw:0,0";
    snd_pcm_uframes_t buffer_size = 4096;
    snd_pcm_uframes_t period_size = 1024;

    if ((err = snd_pcm_open(&pcm_handle, pcm_device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "snd_pcm_open error: %s\n", snd_strerror(err));
        return 1;
    }

    snd_pcm_hw_params_malloc(&hw_params);
    snd_pcm_hw_params_any(pcm_handle, hw_params);
    snd_pcm_hw_params_set_access(pcm_handle, hw_params, SND_PCM_ACCESS_MMAP_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate(pcm_handle, hw_params, 48000, 0);
    snd_pcm_hw_params_set_channels(pcm_handle, hw_params, 2);
    snd_pcm_hw_params_set_buffer_size_near(pcm_handle, hw_params, &buffer_size);
    snd_pcm_hw_params_set_period_size_near(pcm_handle, hw_params, &period_size, 0);

    if ((err = snd_pcm_hw_params(pcm_handle, hw_params)) < 0) {
        fprintf(stderr, "snd_pcm_hw_params error: %s\n", snd_strerror(err));
        goto cleanup;
    }

    if ((err = snd_pcm_prepare(pcm_handle)) < 0) {
        fprintf(stderr, "snd_pcm_prepare error: %s\n", snd_strerror(err));
        goto cleanup;
    }

    const snd_pcm_channel_area_t *areas;
    snd_pcm_uframes_t offset, frames;
    void *buffer;
    int loops = 1000;

    printf("Stress test mmap: scrittura e commit ripetuti...\n");

    for (int i = 0; i < loops; i++) {
        frames = period_size;

        if ((err = snd_pcm_mmap_begin(pcm_handle, &areas, &offset, &frames)) < 0) {
            fprintf(stderr, "snd_pcm_mmap_begin error: %s\n", snd_strerror(err));
            break;
        }

        buffer = (char *)areas->addr + (areas->first / 8) + (offset * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8) * 2;

        // Scrive dati random per stressare la mappa
        for (unsigned int j = 0; j < frames * 2; j++) {
            ((short*)buffer)[j] = (short)(rand() % 32767);
        }

        if ((err = snd_pcm_mmap_commit(pcm_handle, offset, frames)) < 0) {
            fprintf(stderr, "snd_pcm_mmap_commit error: %s\n", snd_strerror(err));
            break;
        }

        // Avvia playback se non già iniziato
        if (i == 0) {
            if ((err = snd_pcm_start(pcm_handle)) < 0) {
                fprintf(stderr, "snd_pcm_start error: %s\n", snd_strerror(err));
                break;
            }
        }
    }

    printf("Stress test completato. Controlla dmesg per eventuali warning PAT.\n");

    sleep(5);  // lascia tempo per messaggi kernel

cleanup:
    snd_pcm_hw_params_free(hw_params);
    snd_pcm_close(pcm_handle);
    return err < 0 ? 1 : 0;
}

