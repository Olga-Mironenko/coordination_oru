import os
import pickle
import re
import textwrap
from dataclasses import dataclass
from typing import Callable

from IPython.display import display, HTML
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.io as pio
import plotly.express as px
from plotly.subplots import make_subplots
from scipy.stats import pearsonr, spearmanr, kendalltau
import scipy.stats as stats
from sklearn.feature_selection import mutual_info_regression

RUNDIRS = '../logs/rundirs'

COL_I = 'i_map'
COL_J = 'position'

# Dictionary to map Map IDs to the number of OPs
MAP_TO_OPS = {
    1: 2, 6: 2, 10: 2,  # Maps with 2 OPs
    2: 1, 3: 1, 4: 1, 5: 1, 7: 1, 8: 1, 9: 1,  # Maps with 1 OP
}

FONTSIZE_LABEL = 18
FONTSIZE_TITLE = 18
FONTSIZE_SUPTITLE = 18
FONTSIZE_LEGEND = 10

TITLE_TO_SUBSCATTER = {}

pio.templates.default = "plotly_dark"


def get_key2df(runname):
    global CACHE_KEY2DF

    try:
        CACHE_KEY2DF
    except NameError:
        CACHE_KEY2DF = {}
    # Note: "When a module is reloaded, its dictionary (containing the module’s global variables) is retained."
    # (https://docs.python.org/3/library/importlib.html#importlib.reload).

    rundir = f'{RUNDIRS}/{runname}'
    filename = f'{rundir}/key2df.pickle'
    mtime = os.path.getmtime(filename)

    pair = CACHE_KEY2DF.get(runname)
    if pair is None or pair[0] != mtime:
        with open(filename, 'rb') as file:
            key2df = pickle.load(file)

        CACHE_KEY2DF[runname] = mtime, key2df

    return CACHE_KEY2DF[runname][1]


# MARGIN = dict(t=100, b=0, l=80, r=0)


def calculate_heatmap_data(*, df, col_i, col_j, col_data):
    return df[[col_i, col_j, col_data]].pivot(
        index=col_i, columns=col_j, values=col_data
    )


def add_ticks(*, fig, offset_fig, col_i, col_j, heatmap_data, row2label=str, idx_x, idx_y, secondary_y=False):
    idx_x += offset_fig
    idx_y += offset_fig * 2
    # print(f'{idx_x=}, {idx_y=}')

    # Apply axis settings to each subplot
    xaxis_key = 'xaxis' + (str(idx_x + 1) if idx_x > 0 else '')
    yaxis_key = 'yaxis' + (str(idx_y + 1) if idx_y > 0 else '')

    # Update axis title based on whether it's a secondary y-axis
    yaxis_title = "no. of OPs" if secondary_y else col_i

    # Set a larger title_standoff for the secondary y-axis to move the label further from the chart
    standoff_value = 10 if secondary_y else 0

    fig.update_layout(**{
        xaxis_key: dict(
            title=dict(
                text=col_j,
                font=dict(
                    size=18,
                ),
            ),
            tickmode="array",
            tickvals=list(heatmap_data.columns),
            ticktext=[f'{x}' for x in heatmap_data.columns],
            title_standoff=7,  # Move x-axis title closer
            automargin=False,
            constrain='domain',
            # margin=MARGIN,
        ),
        yaxis_key: dict(
            title=dict(
                text=yaxis_title,
                font=dict(
                    size=18,
                ),
            ),
            tickmode="array",
            tickvals=list(heatmap_data.index),
            ticktext=[row2label(y) + '&nbsp;'
                      if not secondary_y
                      else '&nbsp;' + row2label(y)
                      for y in heatmap_data.columns],
            autorange="reversed",  # Reverse the y-axis for top-to-bottom ticks
            title_standoff=standoff_value,  # Move y-axis title closer
            automargin=False,
            constrain='domain',
            # margin=MARGIN,
        ),
    })


def add_all_ticks(*, fig, offset_fig, col_i, col_j, heatmap_data, idx, are_bridges):
    # Check if the current map is low or high connectivity
    primary_label = 'L' if not are_bridges else 'H'  # Use 'L' for low connectivity maps

    add_ticks(
        fig=fig, offset_fig=offset_fig, col_i=col_i, col_j=col_j, heatmap_data=heatmap_data,
        idx_x=idx, idx_y=idx * 2,
        row2label=lambda r: f'{r}{primary_label}',
    )

    add_ticks(
        fig=fig, offset_fig=offset_fig, col_i=col_i, col_j=col_j, heatmap_data=heatmap_data,
        idx_x=idx, idx_y=idx * 2 + 1,
        row2label=lambda r: f'{MAP_TO_OPS[r]}',
        secondary_y=True
    )


def set_hovertemplate(*, fig, col_i, col_j, col_data, is_value_in_tooltip):
    hovertemplate = ""
    # hovertemplate += col_i + ": %{y}<br>"  # `%{y}` is interpreted as the label of the RIGHT (secondary) axis
    hovertemplate += col_j + ": %{x}<br>"
    if is_value_in_tooltip:
        hovertemplate += col_data + ": %{z}"
    hovertemplate += "<extra></extra>"

    fig.update_traces(hovertemplate=hovertemplate)


def plot_df_csd(*, title, col_i, col_j, col_data, heatmap_data):
    # Create an interactive heatmap
    fig = px.imshow(
        heatmap_data,
        labels={"x": col_j, "y": col_i, "color": col_data},
        title=title,
    )

    set_hovertemplate(fig=fig, col_i=col_i, col_j=col_j, col_data=col_data, is_value_in_tooltip=True)

    # Add the secondary y-axis for 'no_ops'
    fig.update_layout(
        yaxis2=dict(
            title="no. of OPs",
            overlaying="y",
            side="right",
            tickmode="array",
            tickvals=list(heatmap_data.index),
            ticktext=[f"{2 if y in [1, 6, 10] else 1}" for y in heatmap_data.index],
            automargin=True,
        )
    )

    return fig


def make_subplots_row(titles):
    if len(titles) == 2:
        horizontal_spacing = 0.25 / len(titles)
        width = 500 * len(titles)  # Adjust based on the number of subplots
        height = 500  # Fixed height to make the plots square
    elif len(titles) == 4:
        horizontal_spacing = 0.25 / len(titles)
        width = 460 * len(titles)  # Adjust based on the number of subplots
        height = 500  # Fixed height to make the plots square
    elif len(titles) == 5:
        horizontal_spacing = 0.25 / len(titles)
        width = 460 * len(titles)  # Adjust based on the number of subplots
        height = 500  # Fixed height to make the plots square
    else:
        horizontal_spacing = 0.25 / len(titles)
        width = 500 * len(titles)  # Adjust based on the number of subplots
        height = 500  # Fixed height to make the plots square

    fig = make_subplots(
        rows=1,
        cols=len(titles),
        subplot_titles=[t.capitalize().replace(' mv ', ' MV ') for t in titles],
        specs=[[{'secondary_y': True}] * len(titles)],
        horizontal_spacing=horizontal_spacing,  # Adjust spacing between subplots
    )

    # Set fixed dimensions to ensure square charts
    fig.update_layout(
        autosize=False,
        width=width,
        height=height,
    )

    for annotation in fig['layout']['annotations']:
        annotation['y'] += 0.03  # Move titles higher by increasing y-coordinate
        annotation['font'] = {
            'size': 20,  # Adjust font size
        }

    return fig


def plot_feature(runname, col_data, title, label):
    display(HTML(f'<h1>{title}</h1>'))
    key2df = get_key2df(runname)

    list_are_bridges = [False, True]
    titles = ['Maps with low connectivity', 'Maps with high connectivity']

    # Create subplots
    fig = make_subplots_row(titles)

    list_pairs = []

    for idx, are_bridges in enumerate(list_are_bridges):
        df = key2df[are_bridges, 'conf']
        df['No. of OPs'] = df['i_map'].map(MAP_TO_OPS)

        heatmap_data = calculate_heatmap_data(df=df, col_i=COL_I, col_j=COL_J, col_data=col_data)

        list_pairs.append((titles[idx], heatmap_data))

        # Determine primary label (L for low connectivity, H for high connectivity)
        primary_label = 'L' if not are_bridges else 'H'

        heatmap_fig = plot_df_csd(
            title=f'{titles[idx]}',  # TODO: `subplot_titles`
            col_i=COL_I,
            col_j=COL_J,
            col_data=col_data,
            heatmap_data=heatmap_data,
        )

        # Apply the same layout settings to the individual figure
        heatmap_fig.update_layout(
            title=dict(
                # text=f'{titles[idx]}',
                x=0.57,
                y=0.85,
                xanchor='center',
                yanchor='top',
            ),
            autosize=False,
            width=500,  # Same fixed width as the subplot
            height=500,  # Same fixed height as the subplot
            coloraxis=dict(
                colorscale="Greys",
                colorbar=dict(  # TODO: remove multiple colorbar definitions in this function
                    title="POD score",
                    titlefont=dict(size=16),
                    x=-0.3,
                    titleside="top",
                    thickness=10,
                    len=1.1  # ⬅ Increases the length of the colorbar (default is 0.5)
                )
            ),  # Colorbar on the left

            xaxis=dict(
                title=COL_J,
                tickmode="array",
                tickvals=list(heatmap_data.columns),
                ticktext=[f'{x}' for x in heatmap_data.columns],
                constrain='domain',
                automargin=False,
                # margin=MARGIN,
            ),
            yaxis=dict(
                title=f'{COL_I} ({primary_label})',
                tickmode="array",
                tickvals=list(heatmap_data.index),
                ticktext=[f'{y}{primary_label}' for y in heatmap_data.index],
                autorange="reversed",
                title_standoff=0.0,  # Move the y-axis label closer to the chart
                constrain='domain',
                automargin=False,
                # margin=MARGIN,
            ),
            yaxis2=dict(
                title='no. of OPs',
                overlaying='y',
                side='right',
                tickmode="array",
                tickvals=list(heatmap_data.index),
                ticktext=[f'{2 if y in [1, 6, 10] else 1}' for y in heatmap_data.index],
                constrain='domain',
                automargin=False,
                # margin=MARGIN,
            ),
            margin=dict(l=50, r=50, t=80, b=50)  # Adjust margins for better spacing
        )

        # Add heatmap to the subplot
        for trace in heatmap_fig.data:
            trace.update(coloraxis="coloraxis1")  # Link each subplot to the shared color axis
            trace.update(showscale=True)
            fig.add_trace(trace, row=1, col=idx + 1, secondary_y=False)
            fig.add_trace(trace, row=1, col=idx + 1, secondary_y=True)

        # Add ticks for the subplot
        add_all_ticks(fig=fig, offset_fig=0, col_i='map ID', col_j='position configuration', heatmap_data=heatmap_data,
                      idx=idx, are_bridges=are_bridges)

        # for axis in fig.layout:
        #     if axis.startswith('xaxis') or axis.startswith('yaxis'):
        #         print(axis)

        # Show the individual figure
        # heatmap_fig.show()

    # Show the individual figure separately
    # heatmap_fig.show()

    fig.update_layout(
        coloraxis1=dict(  # shared color axis
            colorscale="Greys",
            colorbar=dict(
                title=label,
                titlefont=dict(size=16),
                x=-0.13,  # Colorbar position for subplots
                titleside="top",
                thickness=10,
                len=1.1  # ⬅ Increases the length of the colorbar (default is 0.5)
            )
        ),
        xaxis=dict(scaleanchor="y"),  # Ensures the x-axis and y-axis are scaled equally
        yaxis=dict(scaleanchor="x"),  # Ensures the y-axis and x-axis are scaled equally
    )

    # Display the figure
    fig.show()

    display(HTML('<hr/>'))

    return list_pairs


COLUMN_TO_IS_THE_MORE_THE_BETTER = {
    'No. of completed missions': True,
    'No. of collisions': False,
    'No. of near-misses': False,
    'Collision rate': False,
}
TITLE_CMP = 'Strategies Comparison'


def rank_dataframes(dataframes, is_the_more_the_better=True):
    ranked_dfs = [pd.DataFrame(index=df.index, columns=df.columns)
                  for df in dataframes]

    # Get the shape of the DataFrames
    num_rows, num_cols = dataframes[0].shape

    # Iterate over each cell in the DataFrame
    for row in range(num_rows):
        for col in range(num_cols):
            # Extract the values across all DataFrames at the same position
            cell_values = (df.iloc[row, col] for df in dataframes)
            series = pd.Series(cell_values)
            ranks = series.rank(method='min', ascending=not is_the_more_the_better)

            # Iterate over each DataFrame
            for df_index, ranked_df in enumerate(ranked_dfs):
                ranked_df.iloc[row, col] = ranks[df_index]

    return ranked_dfs


def compare_ranked_dfs(df_a, df_b, label_a, label_b, *,
                       label_na,
                       color_a, color_b, color_ab, color_na):
    df_cmp = pd.DataFrame(index=df_a.index, columns=df_a.columns)
    df_colors = pd.DataFrame(index=df_a.index, columns=df_a.columns)

    # Get the shape of the DataFrames
    num_rows, num_cols = df_a.shape

    # Iterate over each cell in the DataFrame
    for row in range(num_rows):
        for col in range(num_cols):
            # Extract the values across all DataFrames at the same position
            value_a = df_a.iloc[row, col]
            value_b = df_b.iloc[row, col]

            if pd.isna(value_a) and pd.isna(value_b):
                value_c = label_na
                color_c = color_na
            elif pd.isna(value_b) or value_a < value_b:
                value_c = label_a
                color_c = color_a
            elif pd.isna(value_a) or value_a > value_b:
                value_c = label_b
                color_c = color_b
            else:
                value_c = label_a + label_b
                color_c = color_ab

            df_cmp.iloc[row, col] = value_c
            df_colors.iloc[row, col] = color_c

    return df_cmp, df_colors


def plot_df(*, runname='', title='', col_i, col_j, col_data, heatmap_data, df_ranks, is_value_in_tooltip):
    # Create an interactive heatmap
    fig = px.imshow(
        heatmap_data,
        labels={"x": col_j, "y": col_i, "color": col_data},
        title=f"{runname}: {title}: {col_data}".capitalize(),  # TODO: not used (only `titles` in `make_subplots_row` are used)
        # text_auto=True,
    )

    # fig.data[0].update(text=df_ranks.values, texttemplate="%{text}")

    # Show ranking text only for the "Strategies Comparison" heatmap
    if df_ranks is not None:
        fig.data[0].update(text=df_ranks.values, texttemplate="%{text}")

    set_hovertemplate(fig=fig, col_i=col_i, col_j=col_j, col_data=col_data, is_value_in_tooltip=is_value_in_tooltip)

    return fig


def plot_df_all(
        runname, col_data, *,
        fig, offset_fig, strategies, titles, titles_fig, df,
        are_bridges, label_are_bridges, slowness,
        is_full, is_baseline_only, is_comparison_only
):
    assert is_full + is_baseline_only + is_comparison_only == 1

    col_strategy = 'forcing'
    is_the_more_the_better = COLUMN_TO_IS_THE_MORE_THE_BETTER[col_data]

    title_fig = "" if is_comparison_only else f"{col_data}<br>(slowness: {slowness}; coordination strategies)"

    title_to_heatmap_data = {}
    for idx, strategy in enumerate(strategies):
        heatmap_data = calculate_heatmap_data(
            df=df[df[col_strategy] == strategy],
            col_i=COL_I,
            col_j=COL_J,
            col_data=col_data,
        )
        title_to_heatmap_data[strategy] = heatmap_data
        comment = '' if not title_fig else title_fig.replace('<br>', '\n').splitlines()[-1]
        title_pair = label_are_bridges + (' ' + comment if comment else '') + ': ' + strategy
        yield title_pair, heatmap_data

    if is_full or is_comparison_only:
        strategies_for_ranks = strategies[1:3]
        dfs_for_ranks = [title_to_heatmap_data[strategy] for strategy in strategies_for_ranks]
        dfs_ranks = rank_dataframes(dfs_for_ranks,
                                    is_the_more_the_better=is_the_more_the_better)
        df_cmp, df_color = compare_ranked_dfs(
            *dfs_ranks,
            *(s.split()[-1][0].upper() for s in strategies_for_ranks),
            label_na='N/A',
            color_a=0.0,
            color_b=1.0,
            color_ab=0.5,
            color_na=0.25,
        )
        title_to_heatmap_data[TITLE_CMP] = df_color

        # Modify ranks for comparison heatmap if slowness is "with rerouting"
        if slowness == 'with rerouting':
            df_cmp = df_cmp.replace({'P': 'PR', 'S': 'SR', 'PS': 'PSR'})

    for idx, title in enumerate(titles):
        heatmap_data = title_to_heatmap_data[title]

        heatmap_fig = plot_df(
            runname=runname,
            title=title,
            col_i=COL_I,
            col_j=COL_J,
            col_data=col_data,
            heatmap_data=heatmap_data,
            df_ranks=df_cmp if title == TITLE_CMP else None,
            is_value_in_tooltip=title != TITLE_CMP,
        )

        # Add heatmap to the subplot
        for trace in heatmap_fig.data:
            if title != TITLE_CMP:
                trace.update(coloraxis="coloraxis1")  # Link each subplot to the shared color axis
            else:
                trace.update(coloraxis="coloraxis2", showscale=False)
            trace.update(showscale=title != TITLE_CMP)  # Show colorbar only if it's not the Comparison plot
            for secondary_y in False, True:
                fig.add_trace(trace, row=1, col=idx + 1 + offset_fig, secondary_y=secondary_y)

        add_all_ticks(fig=fig, offset_fig=offset_fig, col_i='map ID', col_j='position configuration',
                      heatmap_data=heatmap_data, idx=idx, are_bridges=are_bridges)

    # Update layout with shared color scale

    fig.update_layout(
        title=dict(
            text=title_fig + '<br>',
            y=0.96,
            font=dict(
                size=20,
            )
        ),
        coloraxis1=dict(  # shared color axis
            colorscale="Greens" if is_the_more_the_better else "Reds",
            colorbar=dict(
                title="<br>".join(textwrap.wrap(col_data, 10)) + "<br>&nbsp;",
                titlefont=dict(size=14),
                x=-0.25 / len(titles_fig),
                titleside="top",
                thickness=10,
                len=1.1  # ⬅ Increases the length of the colorbar (default is 0.5)
            )
        ),
        coloraxis2=dict(
            colorscale=[
                [0.0, '#FFFFE0'],  # 0.0 is P
                [0.25, '#3D3D3D'],  # 0.25 is NA
                [0.5, '#D3D3D3'],  # 0.5 is PS
                [1.0, '#ADD8E6'],  # 1.0 is S
            ],
        )
    )


def make_comparison_title(are_bridges, slowness):
    if slowness == 'baseline':
        comment = 'Violations by MV: priorities'
    else:
        comment = 'Violations by MV: priorities, speed'
    return f'{TITLE_CMP}<br>Maps with {"high" if are_bridges else "low"} connectivity<br>{comment}'


def plot_runname(runname, column, *, title=None, is_baseline_only=False, is_comparison_only=False, is_blocked_to_na=False):
    is_full = not (is_baseline_only or is_comparison_only)
    assert is_full + is_baseline_only + is_comparison_only == 1

    if title is None:
        title = column
    display(HTML(f'<h1>{title}</h1>'))

    if is_baseline_only:
        strategies = titles = ['baseline']
        titles_fig = ['baseline (Maps with low connectivity)', 'baseline (Maps with high connectivity)']
        fig = make_subplots_row(titles_fig)
        offset_fig = 0
    elif is_comparison_only:
        titles = [TITLE_CMP]
        titles_fig = [
            make_comparison_title(are_bridges, slowness)
            for slowness in ('baseline', 'without rerouting', 'with rerouting')
            for are_bridges in (False, True)
            if not (not are_bridges and slowness == 'with rerouting')
        ]
        fig = make_subplots_row(titles_fig)
        offset_fig = 0

    list_pairs = []

    for are_bridges in False, True:
        label_are_bridges = 'Maps with high connectivity' if are_bridges else 'Maps with low connectivity'
        if is_full:
            display(HTML(f'<h2>{label_are_bridges}</h2>'))

        slownesses = ['baseline']
        if is_full or is_comparison_only:
            slownesses += [
                'without rerouting',
                'with rerouting',
            ]

        key2df = get_key2df(runname)
        is_aut = True
        df = key2df[are_bridges, is_aut]
        if is_blocked_to_na:
            df = df.copy()
            df.loc[df['Is blocked'], column] = pd.NA

        for slowness in slownesses:
            dfx = df[~df['passhum'] & (df['slowness'] == slowness)]
            assert dfx.empty == (not are_bridges and slowness == 'with rerouting')
            if dfx.empty:
                continue

            if is_full or is_comparison_only:
                col_strategy = 'forcing'
                strategies = list(dfx[col_strategy].unique())
                if is_full:
                    titles = titles_fig = strategies + [TITLE_CMP]
                    fig = make_subplots_row(titles)
                    offset_fig = 0

            list_pairs += plot_df_all(
                runname,
                column,
                fig=fig,
                offset_fig=offset_fig,
                strategies=strategies,
                titles=titles,
                titles_fig=titles_fig,
                df=dfx,
                are_bridges=are_bridges,
                label_are_bridges=label_are_bridges,
                slowness=slowness,
                is_full=is_full,
                is_baseline_only=is_baseline_only,
                is_comparison_only=is_comparison_only,
            )
            offset_fig += len(titles)

            if is_full:
                fig.show()

    if is_baseline_only or is_comparison_only:
        fig.show()

    display(HTML('<hr/>'))

    return list_pairs


def scatter(ax, xs, ys, *, color, label, label_trendline=None):
    if label_trendline is None:
        label_trendline = 'Trendline'

    if label is not None:
        ax.scatter(xs, ys, s=5, alpha=0.7, color=color, label='\n'.join(textwrap.wrap(label, 15)))

    try:
        coeffs = np.polyfit(xs, ys, 1)
    except np.linalg.LinAlgError:
        pass
    else:
        trendline = np.poly1d(coeffs)
        ax.plot(xs, trendline(xs), color=color, alpha=0.7, linestyle='--', label=label_trendline)


def add_grid_etc(ax, *, title, xlabel, ylabel):
    ax.set_title(title, fontsize=FONTSIZE_TITLE)
    ax.set_xlabel(xlabel, fontsize=FONTSIZE_LABEL)
    ax.set_ylabel(ylabel, fontsize=FONTSIZE_LABEL)
    ax.legend(fontsize=FONTSIZE_LEGEND)
    ax.grid(alpha=0.3)


def compute_mi_p_value(xs, ys, n_permutations=1000):
    xs = np.array(xs).reshape(-1, 1)
    ys = np.array(ys)

    # Compute observed MI
    observed_mi = mutual_info_regression(xs, ys, random_state=1).item()

    # Compute MI under permutation null hypothesis
    permuted_mis = []
    for _ in range(n_permutations):
        np.random.shuffle(ys)  # Shuffle ys randomly
        permuted_mi = mutual_info_regression(xs, ys, random_state=1).item()
        permuted_mis.append(permuted_mi)

    # Compute p-value (fraction of permuted MIs >= observed MI)
    p_value = np.mean(np.array(permuted_mis) >= observed_mi)
    return observed_mi, p_value

def plot_hist_norm(data):
    """Plots histogram, normal curve, and performs normality tests independently."""

    # Create a new figure
    # plt.figure()

    # Plot histogram
    # plt.hist(data, bins=30, density=True, alpha=0.6, color='b', edgecolor='black')

    # Fit a normal distribution
    mu, std = np.mean(data), np.std(data)
    xmin, xmax = plt.xlim()  # Get x-axis limits
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)

    # Plot normal distribution curve
    # plt.plot(x, p, 'r', linewidth=2)

    # Labels
    # plt.title("Histogram with Normal Distribution Fit")
    # plt.xlabel("Value")
    # plt.ylabel("Frequency (Normalized)")

    # Show non-blocking plot
    # plt.show(block=False)

    # Normality Tests
    shapiro_test = stats.shapiro(data)
    ks_test = stats.kstest(data, 'norm', args=(mu, std))

    print(f"Shapiro-Wilk test: Statistic={shapiro_test.statistic:.4f}, p-value={shapiro_test.pvalue:.4f}")
    # print(f"Kolmogorov-Smirnov test: Statistic={ks_test.statistic:.4f}, p-value={ks_test.pvalue:.4f}")
    # print()

    # # Q-Q Plot in a separate figure
    # plt.figure()
    # stats.probplot(data, dist="norm", plot=plt)
    # plt.title("Q-Q Plot")
    #
    # # Show non-blocking Q-Q plot
    # plt.show(block=False)



def show_correlation(ax, title, color, col_x, col_y, xs, ys):
    # Calculate correlations and p-values
    results = []
    name2function = {
        'Spearman': spearmanr,
        'Mutual Information': compute_mi_p_value,
        'Pearson': pearsonr,
        'Kendall': kendalltau,
    }
    for name, func in name2function.items():
        corr, pval = func(xs, ys)
        results.append({"Method": name, "Correlation": corr, "P-Value": pval})

    # Convert results to DataFrame
    df = pd.DataFrame(results)
    df['Correlation'] = df['Correlation'].map(lambda x: f'{x:.3f}')
    df['P-Value'] = df['P-Value'].map(lambda x: f'{x:.1e}')
    display(df)

    plot_hist_norm(ys)

    # Plot scatter plot with linear trendline (for Pearson correlation)
    def subscatter(*, ax, color, label, label_trendline=None):
        scatter(ax, xs, ys, color=color, label=label, label_trendline=label_trendline)

    TITLE_TO_SUBSCATTER[title] = subscatter

    subscatter(ax=ax, color=color, label='Scenarios')

    # Customize plot
    pretitle, posttitle = split_title(title)
    add_grid_etc(ax, title=f'{posttitle}\n{pretitle}', xlabel=col_x, ylabel=col_y)


def split_title(title):
    match = re.match(r'(.+) [(](.+)[)]$', title)
    assert match is not None
    return match.groups()


def show_correlation_comparison(ax, col_x, col_y, title,
                                xs_base, ys_base,
                                label1, xs1, ys1,
                                label2, xs2, ys2):
    if xs_base is not None:
        color_base = 'gray'
        color1 = 'red'
        color2 = 'blue'
    else:
        color_base = None
        color1 = 'gray'
        color2 = 'green'

    if xs_base is not None:
        scatter(ax, xs_base, ys_base, color=color_base, label=None, label_trendline='Baseline trendline')

    assert label1 != label2
    scatter(ax, xs1, ys1, color=color1, label=label1)
    scatter(ax, xs2, ys2, color=color2, label=label2)

    # Customize plot
    add_grid_etc(ax, title=title, xlabel=col_x, ylabel=col_y)


def make_subplots_matplotlib(n_plots):
    fig, [axes] = plt.subplots(1, n_plots, squeeze=False,
                                figsize=(n_plots * 4, 4), sharey='all')
    return fig, axes


def show_correlations(col_x: str,
                      col_y: str,
                      pairs1: list[tuple[str, pd.DataFrame]],
                      pairs2: list[tuple[str, pd.DataFrame]]) -> Callable[[matplotlib.axes.Axes, str, str], None] | None:
    display(HTML(f'<h2>{col_x} vs. {col_y}</h2>'))

    assert len(pairs1) == len(pairs2)
    n_pairs = len(pairs1)
    is_comparison = n_pairs > 1
    n_plots = n_pairs + 1 if is_comparison else n_pairs
    fig, axes = make_subplots_matplotlib(n_plots)

    colors = ['black', 'red', 'blue']

    titles = []
    list_xs = []
    list_ys = []
    for i, (ax, (title_x, df_x), (title_y, df_y)) in enumerate(zip(axes, pairs1, pairs2, strict=False)):
        title = title_x if title_x == title_y else f'{title_x} ({title_y})'
        titles.append(title)

        xs = df_x.to_numpy().flatten()
        list_xs.append(xs)

        ys = df_y.to_numpy().flatten()
        list_ys.append(ys)

        show_correlation(ax, title, colors[i], col_x, col_y if i == 0 else '', xs, ys)

    if not is_comparison:
        show_comparison = None
    else:
        i1 = n_pairs - 2
        i2 = n_pairs - 1
        pretitle1, posttitle1 = split_title(titles[i1])
        pretitle2, posttitle2 = split_title(titles[i2])
        assert pretitle1 == pretitle2
        title = f'Strategies comparison\n{pretitle1}'

        i_base = n_pairs - 3
        if i_base < 0:
            xs_base = ys_base = None
        else:
            xs_base = list_xs[i_base]
            ys_base = list_ys[i_base]

        def show_comparison(ax, col_y, title):
            show_correlation_comparison(ax, col_x, col_y, title,
                                        xs_base, ys_base,
                                        posttitle1, list_xs[i1], list_ys[i1],
                                        posttitle2, list_xs[i2], list_ys[i2])

        show_comparison.col_y = col_y
        show_comparison.pretitle = pretitle1
        show_comparison(axes[-1], '', title)

    plt.tight_layout()  # Adjust layout to prevent overlap
    plt.show()
    # display(HTML('<hr/>'))

    return show_comparison


def filter_pairs(pairs, titles):
    titles_lower = [t.lower() for t in titles]
    return [p for p in pairs if p[0].lower() in titles_lower]


@dataclass
class RuleViolationSection:
    runname: str
    title_to_pairs_feature: dict[str, list[tuple[str, pd.DataFrame]]]
    connectivity: str
    col_data: str

    title: str
    title2paramdict: dict[str, dict[str, str]]
    is_comparison: bool = True

    def are_bridges(self):
        assert self.connectivity in ('low', 'high')
        return self.connectivity == 'high'

    def is_the_more_the_better(self):
        return COLUMN_TO_IS_THE_MORE_THE_BETTER[self.col_data]

    def filter_df(self, paramdict):
        key2df = get_key2df(self.runname)
        is_aut = True
        df = key2df[self.are_bridges(), is_aut]

        mask = pd.Series(True, index=df.index)  # Start with all True
        for col, val in paramdict.items():
            mask &= df[col] == val

        filtered_df = df[mask]
        assert not filtered_df.empty, paramdict
        return filtered_df

    def make_title_to_heatmap_data(self, titles_fig):
        title_to_heatmap_data = {}
        list_pairs = []
        for title, paramdict in self.title2paramdict.items():
            heatmap_data = calculate_heatmap_data(
                df=self.filter_df(paramdict),
                col_i=COL_I,
                col_j=COL_J,
                col_data=self.col_data,
            )
            title_to_heatmap_data[title] = heatmap_data

            title_pair = title
            list_pairs.append((title_pair, heatmap_data))

        if not self.is_comparison:
            df_cmp = None
        else:
            titles_for_ranks = titles_fig[-3:-1]
            assert len(titles_for_ranks) == 2
            dfs_for_ranks = [title_to_heatmap_data[title] for title in titles_for_ranks]
            dfs_ranks = rank_dataframes(dfs_for_ranks, is_the_more_the_better=self.is_the_more_the_better())
            df_cmp, df_color = compare_ranked_dfs(
                *dfs_ranks,
                *(s.split()[-1][0].upper() for s in titles_for_ranks),
                label_na='N/A',
                color_a=0.0,
                color_b=1.0,
                color_ab=0.5,
                color_na=0.25,
            )
            title_to_heatmap_data[TITLE_CMP] = df_color

        return title_to_heatmap_data, df_cmp, list_pairs

    def render(self) -> dict[str, Callable[[matplotlib.axes.Axes, str, str], None] | None]:
        display(HTML(f'<h2><b>{self.title}</b> ({self.col_data}, {self.connectivity} connectivity)</h2>'))

        titles_fig = list(self.title2paramdict)
        if self.is_comparison:
            titles_fig.append(TITLE_CMP)
        fig = make_subplots_row(titles_fig)

        title_to_heatmap_data, df_cmp, list_pairs = self.make_title_to_heatmap_data(titles_fig)

        for idx, title in enumerate(titles_fig):
            heatmap_data = title_to_heatmap_data[title]

            heatmap_fig = plot_df(
                col_i=COL_I,
                col_j=COL_J,
                col_data=self.col_data,
                heatmap_data=heatmap_data,
                df_ranks=df_cmp if title == TITLE_CMP else None,
                is_value_in_tooltip=title != TITLE_CMP,
            )

            # Add heatmap to the subplot
            for trace in heatmap_fig.data:
                if title != TITLE_CMP:
                    trace.update(coloraxis="coloraxis1")  # Link each subplot to the shared color axis
                else:
                    trace.update(coloraxis="coloraxis2", showscale=False)
                trace.update(showscale=title != TITLE_CMP)  # Show colorbar only if it's not the Comparison plot
                for secondary_y in False, True:
                    fig.add_trace(trace, row=1, col=idx + 1, secondary_y=secondary_y)

            add_all_ticks(fig=fig, offset_fig=0, col_i='map ID', col_j='position configuration',
                          heatmap_data=heatmap_data, idx=idx, are_bridges=self.are_bridges())

        self.render_fig(fig, titles_fig)

        title_to_show_comparison = {}
        for title_features, pairs_feature in self.title_to_pairs_feature.items():
            list_pairs_csd = [pairs_feature[0 if self.connectivity == 'low' else 1]] * len(list_pairs)
            show_comparison = show_correlations(title_features, self.col_data,
                                                list_pairs_csd, list_pairs)
            if show_comparison is not None:
                show_comparison.title = self.title
            title_to_show_comparison[title_features] = show_comparison

        display(HTML('<hr/>'))
        return title_to_show_comparison

    def render_fig(self, fig, titles_fig):
        # Update layout with shared color scale:
        fig.update_layout(
            title=dict(
                text=self.col_data + '<br>',
                y=0.96,
                font=dict(
                    size=20,
                )
            ),
            coloraxis1=dict(  # shared color axis
                colorscale="Greens" if self.is_the_more_the_better() else "Reds",
                colorbar=dict(
                    title="<br>".join(textwrap.wrap(self.col_data, 10)) + "<br>&nbsp;",
                    titlefont=dict(size=14),
                    x=-0.25 / len(titles_fig),
                    titleside="top",
                    thickness=10,
                    len=1.1  # ⬅ Increases the length of the colorbar (default is 0.5)
                )
            ),
            coloraxis2=dict(
                colorscale=[
                    [0.0, '#FFFFE0'],   # 0.0 is P
                    [0.25, '#3D3D3D'],  # 0.25 is NA
                    [0.5, '#D3D3D3'],   # 0.5 is PS
                    [1.0, '#ADD8E6'],   # 1.0 is S
                ],
            )
        )

        fig.show()


def render_metric2connectivity2shows(
        metric2connectivity2shows: dict[
            str,
            dict[
                str,
                list[
                    dict[
                        str,
                        Callable[[matplotlib.axes.Axes, str, str], None] | None
                    ]
                ]
            ]
        ]
) -> None:
    for i_title in range(2):
        def get_show(title_to_show_comparison):
            return list(title_to_show_comparison.values())[i_title]

        for col_data, connectivity2shows in metric2connectivity2shows.items():
            display(HTML(f'<h1>{col_data}</h1>'))
            pairs_shows = zip(*connectivity2shows.values(), strict=True)  # [[..., ...], ..., [..., ...]]

            shows_summary = []
            for i_pair, pair in enumerate(pairs_shows):
                shows = [get_show(show)
                         for show in pair
                         if show is not None
                         and get_show(show) is not None]
                assert shows
                section = shows[0].title
                assert all(show.title == section for show in shows)

                fig, axes = make_subplots_matplotlib(len(shows))
                fig.suptitle(section, fontsize=FONTSIZE_SUPTITLE)
                for i_show, (ax, show_comparison) in enumerate(zip(axes, shows, strict=True)):
                    show_comparison(ax, show_comparison.col_y if i_show == 0 else '', show_comparison.pretitle)
                    if i_pair != 1:  # "Speed violation"
                        shows_summary.append(show_comparison)
                plt.tight_layout(rect=[0, 0, 1, 0.99])
                plt.show()

            fig, axes = make_subplots_matplotlib(len(shows_summary))
            for i_show, (ax, show_comparison) in enumerate(zip(axes, shows_summary, strict=True)):
                show_comparison(ax, show_comparison.col_y if i_show == 0 else '', show_comparison.pretitle)
            plt.tight_layout()
            plt.show()


def render_metric2title2subscatter(metric2title2subscatter):
    col_x = 'POD scores'

    for col_data, title2subscatter in metric2title2subscatter.items():
        display(HTML(f'<h1>{col_data}</h1>'))

        fig, (ax1, ax2, ax3) = make_subplots_matplotlib(3)

        for ax, conn in (ax1, 'low'), (ax2, 'high'):
            title2subscatter[f'Maps with {conn} connectivity (Baseline prime 2)'](
                ax=ax, color='black', label='Baseline prime 2')
            title2subscatter[f'Maps with {conn} connectivity (Baseline prime 1)'](
                ax=ax, color='lightgray', label=None, label_trendline='Baseline prime 1')
            title2subscatter[f'Maps with {conn} connectivity (Baseline prime 3)'](
                ax=ax, color='darkgray', label=None, label_trendline='Baseline prime 3')
            add_grid_etc(ax, title=f'Maps with {conn} connectivity', xlabel=col_x, ylabel=col_data)

        # TODO:
        # title2subscatter[f'Maps with {conn} connectivity (Baseline prime 2)'](
        #     ax=ax, color='black', label='Baseline prime 2')
        # title2subscatter[f'Maps with {conn} connectivity (Baseline prime 1)'](
        #     ax=ax, color='lightgray', label=None, label_trendline='Baseline prime 1')
        # title2subscatter[f'Maps with {conn} connectivity (Baseline prime 3)'](
        #     ax=ax, color='darkgray', label=None, label_trendline='Baseline prime 3')
        # add_grid_etc(ax, title=f'Maps with {conn} connectivity', xlabel=col_x, ylabel=col_data)

        plt.tight_layout()
        plt.show()


def version3(runname):
    title_to_pairs_feature = {
        'POD scores':
            plot_feature(runname, 'POD score (AVs)', 'POD scores', 'POD score'),
        'No. of OPs':
            plot_feature(runname, 'No. of OPs', 'No. of OPs', 'No. of OPs'),
    }

    metric2connectivity2shows = {}
    metric2title2subscatter = {}

    for col_data in 'No. of completed missions', 'No. of collisions', 'Collision rate':
        connectivity2shows = metric2connectivity2shows[col_data] = {}
        TITLE_TO_SUBSCATTER.clear()

        for connectivity in 'low', 'high':
            display(HTML(f'<h1>{col_data} ({connectivity} connectivity)</h1>'))

            shows = connectivity2shows[connectivity] = []

            shows.append(
                RuleViolationSection(
                    runname=runname,
                    title_to_pairs_feature=title_to_pairs_feature,
                    connectivity=connectivity,
                    col_data=col_data,

                    title='Priorities violation',
                    title2paramdict={
                        'Baseline prime 1':
                            {'slowness': 'baseline', 'forcing': 'ignoring human'},
                        'Dynamic change of priorities':
                            {'slowness': 'baseline', 'forcing': 'change of priorities'},
                        'Stops':
                            {'slowness': 'baseline', 'forcing': 'stops'},
                    },
                ).render()
            )

            shows.append(
                RuleViolationSection(
                    runname=runname,
                    title_to_pairs_feature=title_to_pairs_feature,
                    connectivity=connectivity,
                    col_data=col_data,

                    title='Speed violation',
                    title2paramdict={
                        'Baseline prime 2':
                            {'slowness': 'without rerouting', 'forcing': 'baseline'},
                    } | (
                        {} if connectivity == 'low' else {
                            'Rerouting':
                                {'slowness': 'with rerouting', 'forcing': 'baseline'},
                        }
                    ),
                    is_comparison=False,
                ).render()
            )

            shows.append(
                RuleViolationSection(
                    runname=runname,
                    title_to_pairs_feature=title_to_pairs_feature,
                    connectivity=connectivity,
                    col_data=col_data,

                    title='Priorities violation and Speed violation (Part 1)',
                    title2paramdict={
                        'Baseline prime 3':
                            {'slowness': 'without rerouting', 'forcing': 'ignoring human'},
                        'Dynamic change of priorities':
                            {'slowness': 'without rerouting', 'forcing': 'change of priorities'},
                        'Stops':
                            {'slowness': 'without rerouting', 'forcing': 'stops'},
                    },
                ).render()
            )

            shows.append(
                None if connectivity != 'high' else
                RuleViolationSection(
                    runname=runname,
                    title_to_pairs_feature=title_to_pairs_feature,
                    connectivity=connectivity,
                    col_data=col_data,

                    title='Priorities violation and Speed violation (Part 2)',
                    title2paramdict={
                        'Baseline prime 3':
                            {'slowness': 'without rerouting', 'forcing': 'ignoring human'},
                        'Rerouting and Dynamic change of priorities':
                            {'slowness': 'with rerouting', 'forcing': 'change of priorities'},
                        'Rerouting and Stops':
                            {'slowness': 'with rerouting', 'forcing': 'stops'},
                    },
                ).render()
            )

        print(*TITLE_TO_SUBSCATTER, sep='\n')
        metric2title2subscatter[col_data] = TITLE_TO_SUBSCATTER.copy()

    return metric2connectivity2shows, metric2title2subscatter


def version2(runname):
    # pairs_csd_scores = plot_csd_scores(runname)
    #
    # display(HTML(f'<h1><b><u>Baselines analysis</u></b></h1>'))
    # # for is_blocked_to_na in False, True:
    # #     display(HTML(f'<h1><b>{is_blocked_to_na=}</b></h1>'))
    # is_blocked_to_na = False
    # for col in 'No. of completed missions', 'No. of collisions', 'No. of near-misses', 'Collision rate':
    #     # for prime in 1, 2, 3:
    #     #     # TODO: {col}, baseline prime {prime}, low/high, is_blocked_to_na=is_blocked_to_na
    #     #     pairs_missions_baseline = (
    #     #         plot_runname(runname, col, prime=prime, is_baseline_only=True)
    #     #     )
    #     pairs_missions_baseline = (
    #         plot_runname(runname, col,
    #                      # title=f'{col} ({is_blocked_to_na=})',
    #                      is_baseline_only=True, is_blocked_to_na=is_blocked_to_na)
    #     )
    #
    #     if col not in ('No. of collisions', 'No. of near-misses'):
    #         show_correlations('POD scores', col,
    #                           pairs_csd_scores, pairs_missions_baseline)  # low, high
    #
    #     display(HTML('<hr/>'))

    display(HTML(f'<h1><b><u>Analysis for all</u></b></h1>'))
    # for is_blocked_to_na in False, True:
    for col in 'No. of completed missions', 'No. of collisions', 'No. of near-misses', 'Collision rate':
        pairs_missions_all = plot_runname(runname, col)

        # plot_runname(runname, col, is_comparison_only=True)
        #
        # if col not in ('No. of collisions', 'No. of near-misses'):
        #     # for each of the 15 heatmaps:
        #     #     show_correlations('POD scores', heatmap)
        #     ...


def version1(runname):
    pairs_csd_scores = plot_feature(runname, 'POD score (AVs)', 'POD scores', 'POD score')

    pairs_missions_baseline = (
        plot_runname(runname, 'No. of completed missions', is_baseline_only=True)
    )
    show_correlations('POD scores', 'No. of completed missions',
                      pairs_csd_scores, pairs_missions_baseline)
    pairs_missions_all = plot_runname(runname, 'No. of completed missions')
    # print(*[t for t, _ in pairs_missions_all], sep='\n')

    pairs_missions_without_rerouting = filter_pairs(
        pairs_missions_all,
        [f'Maps with {x} connectivity (slowness: without rerouting; coordination strategies): baseline'
         for x in ('low', 'high')]
    )
    show_correlations('POD scores', 'No. of completed missions',
                      pairs_csd_scores, pairs_missions_without_rerouting)

    plot_runname(runname, 'Collision rate', is_comparison_only=True)
    pairs_collisions_rate_all = plot_runname(runname, 'Collision rate')
    # print(*[t for t, _ in pairs_collisions_rate_all], sep='\n')

    titles_change_of_priorities = [
        f'Maps with {x} connectivity (slowness: baseline; coordination strategies): change of priorities'
         for x in ('low', 'high')
    ]
    pairs_collisions_rate_for_corr = filter_pairs(pairs_collisions_rate_all, titles_change_of_priorities)
    show_correlations('POD scores', 'Collision rate', pairs_csd_scores, pairs_collisions_rate_for_corr)

    plot_runname(runname, 'No. of completed missions', is_comparison_only=True, is_blocked_to_na=True)

    pairs_collisions_all = plot_runname(runname, 'No. of collisions')
    print(*[t for t, _ in pairs_collisions_all], sep='\n')
    pairs_collisions_for_corr = filter_pairs(pairs_collisions_all, titles_change_of_priorities)
    show_correlations('POD scores', 'No. of collisions',
                      pairs_csd_scores, pairs_collisions_for_corr)

    plot_runname(runname, 'No. of collisions', is_comparison_only=True)
    plot_runname(runname, 'No. of near-misses')

    #plot_runname(runname, 'No. of completed missions', is_baseline_only=True)


def main():
    runname = '20250128_094430'
    # version1(runname)
    # version2(runname)
    metric2connectivity2shows, metric2title2subscatter = version3(runname)
    render_metric2connectivity2shows(metric2connectivity2shows)
    render_metric2title2subscatter(metric2title2subscatter)


if __name__ == '__main__':
    main()