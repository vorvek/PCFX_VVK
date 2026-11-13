// CD-DA unit testbench main loop and DPI functions

#include "verilated.h"
#include "Vcdda_tb.h"
#include "svdpi.h"
#include "Vcdda_tb__Dpi.h"
#include "mister_main/support/pcfx/pcfx.h"

int main(int argc, char** argv, char**) {
    const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
    contextp->commandArgs(argc, argv);
    const std::unique_ptr<Vcdda_tb> topp{new Vcdda_tb{contextp.get(), ""}};

    while (VL_LIKELY(!contextp->gotFinish())) {
        topp->eval();
        if (!topp->eventsPending()) break;
        contextp->time(topp->nextTimeSlot());
    }
    topp->final();
    contextp->statsPrintSummary();
    return 0;
}

extern "C" {
svBit pcfx_mount_cd()
{
    pcfx_mount_cd(3, "cd.chd");
    return (pcfx_get_img_size() == 0) ? sv_0 : sv_1;
}

int pcfx_cd_img_size()
{
    return pcfx_get_img_size();
}

int pcfx_read_cd(const svOpenArrayHandle buffer, int lba, int cnt)
{
    void *aptr = svGetArrayPtr(buffer);
    pcfx_read_cd(reinterpret_cast<uint8_t*>(aptr), lba, cnt);
    return 0;
}
} // extern "C"

// compile-command: see .dev/build_cdda_tb.sh
