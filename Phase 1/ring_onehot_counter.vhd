library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Testbench for ring_onehot_counter (rtl)
-- Features:
--  - Clock generator
--  - Synchronous reset
--  - Stimulus: rotate left sequence, then rotate right
--  - Assertions/checker:
--      * q must be one-hot (exactly one '1')
--      * q follows expected sequence on step pulses
--      * q does not change when step='0'
--  - Reports PASS or FAIL at end

entity tb_ring is
end entity tb_ring;

architecture sim of tb_ring is
  -- DUT ports
  signal clk  : std_logic := '0';
  signal rst  : std_logic := '1';
  signal step : std_logic := '0';
  signal dir  : std_logic := '1';
  signal q    : std_logic_vector(3 downto 0);

  constant clk_period : time := 10 ns;

  -- checker variables
  signal prev_q : std_logic_vector(3 downto 0) := "0001";
  signal expected_q : std_logic_vector(3 downto 0) := "0001";
  signal errors : integer := 0;

begin

  -- Instantiate DUT (assumes entity name ring_onehot_counter, arch rtl)
  uut: entity work.ring_onehot_counter(rtl)
    port map (
      clk => clk,
      rst => rst,
      step => step,
      dir => dir,
      q => q
    );

  -- clock process
  clk_proc: process
  begin
    while now < 1 ms loop
      clk <= '0';
      wait for clk_period/2;
      clk <= '1';
      wait for clk_period/2;
    end loop;
    wait;
  end process clk_proc;

  -- stimulus process
  stim_proc: process
  begin
    -- initial reset
    rst <= '1';
    wait for 25 ns;  -- hold reset for some cycles
    rst <= '0';
    wait for clk_period;

    -- simple rotate-left test (8 steps)
    dir <= '1';
    for i in 0 to 7 loop
      -- prepare step synchronous to rising edge: assert step before rising edge
      step <= '1';
      wait for clk_period; -- step sampled on rising edge
      step <= '0';
      wait for clk_period;
    end loop;

    -- short pause
    wait for 5 * clk_period;

    -- rotate-right test (4 steps)
    dir <= '0';
    for i in 0 to 3 loop
      step <= '1';
      wait for clk_period;
      step <= '0';
      wait for clk_period;
    end loop;

    -- test that q does not change when step=0 (wait 4 cycles)
    step <= '0';
    expected_q <= q; -- snapshot
    wait for 4 * clk_period;
    if q /= expected_q then
      report "ERROR: q changed while step=0" severity error;
      errors <= errors + 1;
    end if;

    -- final report
    wait for clk_period;
    if errors = 0 then
      report "TESTBENCH PASSED: no assertion failures" severity note;
    else
      report "TESTBENCH FAILED: " & integer'image(errors) & " errors" severity warning;
    end if;

    wait for 20 * clk_period;
    wait;
  end process stim_proc;

  -- checker process: runs at every rising edge and validates one-hot & expected sequence
  checker_proc: process(clk)
  begin
    if rising_edge(clk) then
      -- one-hot check
      if q = "0000" then
        report "ERROR: q has zero ones (0000)" severity error;
        errors <= errors + 1;
      elsif (q(0) + q(1) + q(2) + q(3)) /= '1' then
        -- VHDL doesn't allow arithmetic on std_logic directly; check using case
        null; -- handled below
      end if;

      -- explicit one-hot count check
      case q is
        when "0001" | "0010" | "0100" | "1000" => null; -- OK
        when others =>
          report "ERROR: q not one-hot (" & q & ")" severity error;
          errors <= errors + 1;
      end case;

      -- expected-sequence check: only validate when step was asserted across the previous rising edge
      -- Compare to expected_q which we update when we observe step='1'
      if step = '1' then
        -- compute expected next state based on dir and prev expected
        if dir = '1' then
          -- rotate left
          expected_q <= expected_q(2 downto 0) & expected_q(3);
        else
          -- rotate right
          expected_q <= expected_q(0) & expected_q(3 downto 1);
        end if;
        -- check DUT output equals expected
        if q /= expected_q then
          report "ERROR: unexpected q. expected " & expected_q & " observed " & q severity error;
          errors <= errors + 1;
        end if;
      end if;

      -- track prev_q
      prev_q <= q;
    end if;
  end process checker_proc;

end architecture sim;
