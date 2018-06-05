#--------------------------------------------------------------
# 
#  Project #5: Optimizing performance on a pipelined processor
#
#  June 5, 2018
#
#  Jin-Soo Kim (jinsoo.kim@snu.ac.kr)
#  Systems Software & Architecture Laboratory
#  Dept. of Computer Science and Engineering
#  Seoul National University
#
#--------------------------------------------------------------

CAT = /bin/cat

MISCDIR = ../misc
PIPEDIR = ../pipe

YAS = $(MISCDIR)/yas
PSIM = $(PIPEDIR)/psim
PSIMFLAGS = -s

TARGET	= bmptest.yo
ASRCS	= bmptest.ys
YSRCS	= bmpmain.ys bmpgrid.ys
MEMOUT  = memory.out
MEMANS  = result.out

all: $(TARGET)

$(TARGET): $(ASRCS)
	$(YAS) $(ASRCS) $@

$(ASRCS): $(YSRCS)
	$(CAT) $(YSRCS) > $(ASRCS)

clean:
	$(RM) -f $(TARGET) $(ASRCS) $(MEMOUT) *~

run: $(TARGET)
	$(PSIM) $(TARGET)

testrun: $(TARGET)
	$(PSIM) $(PSIMFLAGS) $(TARGET)

test: 
	@rm -f $(MEMOUT)
	@make testrun
	@diff $(MEMOUT) $(MEMANS) && echo "Test: SUCCESS"
