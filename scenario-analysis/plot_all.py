import os
import pickle
import textwrap
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.io as pio
import plotly.express as px
from plotly.subplots import make_subplots
from IPython.display import display, HTML

from scipy.stats import pearsonr, spearmanr, kendalltau

RUNDIRS = '../logs/rundirs'

COL_I = 'i_map'
COL_J = 'position'

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
    # Dictionary to map Map IDs to the number of OPs
    map_to_ops = {
        1: 2, 6: 2, 10: 2,  # Maps with 2 OPs
        2: 1, 3: 1, 4: 1, 5: 1, 7: 1, 8: 1, 9: 1,  # Maps with 1 OP
    }

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
        row2label=lambda r: f'{map_to_ops[r]}',
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


def plot_csd_scores(runname):
    col_data = 'CSD score (AVs)'

    display(HTML(f'<h1>POD scores</h1>'))
    key2df = get_key2df(runname)

    list_are_bridges = [False, True]
    titles = ['Maps with low connectivity', 'Maps with high connectivity']

    # Create subplots
    fig = make_subplots_row(titles)

    list_pairs = []

    for idx, are_bridges in enumerate(list_are_bridges):
        df = key2df[are_bridges, 'conf']
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
        add_all_ticks(fig=fig, offset_fig=0, col_i='map ID', col_j='positions configuration', heatmap_data=heatmap_data,
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
                title="POD score",
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
    'Collisions rate': False,
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

        add_all_ticks(fig=fig, offset_fig=offset_fig, col_i='map ID', col_j='positions configuration',
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


def show_correlation(col1, col2, arr1, arr2):
    # Calculate correlations and p-values
    results = []
    name2function = {
        'Pearson': pearsonr,
        'Spearman': spearmanr,
        'Kendall': kendalltau,
    }
    for name, func in name2function.items():
        corr, pval = func(arr1, arr2)
        results.append({"Method": name, "Correlation": corr, "P-Value": pval})

    # Convert results to DataFrame
    df = pd.DataFrame(results)
    df['Correlation'] = df['Correlation'].map(lambda x: f'{x:.3f}')
    df['P-Value'] = df['P-Value'].map(lambda x: f'{x:.1e}')
    display(df)

    # Plot scatter plot with linear trendline (for Pearson correlation)
    plt.figure(figsize=(8, 6))
    plt.scatter(arr1, arr2, alpha=0.7, edgecolor='k', label='Data points')
    coeffs = np.polyfit(arr1, arr2, 1)
    trendline = np.poly1d(coeffs)
    plt.plot(arr1, trendline(arr1), color='red', linestyle='--', label='Linear trendline')

    # Customize plot
    # plt.title('Scatter Plot with Linear Trendline')
    plt.xlabel(col1, fontsize=16)
    plt.ylabel(col2, fontsize=16)
    plt.legend()
    plt.grid(alpha=0.3)
    plt.show()


def show_correlations(col1, col2, pairs1, pairs2):
    display(HTML(f'<h2>{col1} vs. {col2}</h2>'))

    for i, ((title1, m1), (title2, m2)) in enumerate(zip(pairs1, pairs2, strict=True), 1):
        title = title1 if title1 == title2 else f'{title1} ({title2})'
        display(HTML(f'<h3>{title}</h3>'))
        show_correlation(col1, col2, *(m.to_numpy().flatten() for m in (m1, m2)))

    # display(HTML('<hr/>'))


def filter_pairs(pairs, titles):
    titles_lower = [t.lower() for t in titles]
    return [p for p in pairs if p[0].lower() in titles_lower]


@dataclass
class RuleViolationSection:
    runname: str
    pairs_csd_scores: list[tuple[str, pd.DataFrame]]
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

    def render(self):
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

            add_all_ticks(fig=fig, offset_fig=0, col_i='map ID', col_j='positions configuration',
                          heatmap_data=heatmap_data, idx=idx, are_bridges=self.are_bridges())

        self.render_fig(fig, titles_fig)

        list_pairs_csd = [self.pairs_csd_scores[0 if self.connectivity == 'low' else 1]] * len(list_pairs)
        show_correlations('POD scores', self.col_data,
                          list_pairs_csd, list_pairs)
        display(HTML('<hr/>'))

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


def version3(runname):
    pairs_csd_scores = plot_csd_scores(runname)

    for col_data in 'No. of completed missions', 'No. of collisions', 'Collisions rate':
        for connectivity in 'low', 'high':
            display(HTML(f'<h1>{col_data} ({connectivity} connectivity)</h1>'))

            RuleViolationSection(
                runname=runname,
                pairs_csd_scores=pairs_csd_scores,
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

            RuleViolationSection(
                runname=runname,
                pairs_csd_scores=pairs_csd_scores,
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

            RuleViolationSection(
                runname=runname,
                pairs_csd_scores=pairs_csd_scores,
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

            if connectivity == 'high':
                RuleViolationSection(
                    runname=runname,
                    pairs_csd_scores=pairs_csd_scores,
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


def version2(runname):
    # pairs_csd_scores = plot_csd_scores(runname)
    #
    # display(HTML(f'<h1><b><u>Baselines analysis</u></b></h1>'))
    # # for is_blocked_to_na in False, True:
    # #     display(HTML(f'<h1><b>{is_blocked_to_na=}</b></h1>'))
    # is_blocked_to_na = False
    # for col in 'No. of completed missions', 'No. of collisions', 'No. of near-misses', 'Collisions rate':
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
    for col in 'No. of completed missions', 'No. of collisions', 'No. of near-misses', 'Collisions rate':
        pairs_missions_all = plot_runname(runname, col)

        # plot_runname(runname, col, is_comparison_only=True)
        #
        # if col not in ('No. of collisions', 'No. of near-misses'):
        #     # for each of the 15 heatmaps:
        #     #     show_correlations('POD scores', heatmap)
        #     ...


def version1(runname):
    pairs_csd_scores = plot_csd_scores(runname)

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

    plot_runname(runname, 'Collisions rate', is_comparison_only=True)
    pairs_collisions_rate_all = plot_runname(runname, 'Collisions rate')
    # print(*[t for t, _ in pairs_collisions_rate_all], sep='\n')

    titles_change_of_priorities = [
        f'Maps with {x} connectivity (slowness: baseline; coordination strategies): change of priorities'
         for x in ('low', 'high')
    ]
    pairs_collisions_rate_for_corr = filter_pairs(pairs_collisions_rate_all, titles_change_of_priorities)
    show_correlations('POD scores', 'Collisions rate', pairs_csd_scores, pairs_collisions_rate_for_corr)

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
    runname = '20241230_173555'
    # version1(runname)
    # version2(runname)
    version3(runname)


if __name__ == '__main__':
    main()